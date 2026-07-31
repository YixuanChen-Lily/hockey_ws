#!/usr/bin/env python3

import math
from threading import Event, Lock
from typing import Any, Dict, Optional

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robomaster_msgs.action import MoveArm as RobomasterMoveArm

from hockey_interfaces.action import MoveArm


class MoveArmServer(Node):
    """Forward project arm goals to the RoboMaster onboard arm action."""

    def __init__(self) -> None:
        super().__init__("move_arm_server")

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("action_name", "control_arm")
        self.declare_parameter("driver_action_name", "move_arm")
        self.declare_parameter("driver_wait_timeout_sec", 150.0)

        self.robot_id = int(self.get_parameter("robot_id").value)
        self.action_name = str(self.get_parameter("action_name").value)
        driver_action_name = str(
            self.get_parameter("driver_action_name").value
        )
        self.driver_action_name = (
            driver_action_name
            if driver_action_name.startswith("/")
            else f"/robot{self.robot_id}/{driver_action_name}"
        )
        self.driver_wait_timeout_sec = float(
            self.get_parameter("driver_wait_timeout_sec").value
        )

        self._goal_lock = Lock()
        self._goal_active = False
        self._callback_group = ReentrantCallbackGroup()
        self._driver_client = ActionClient(
            self,
            RobomasterMoveArm,
            self.driver_action_name,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            MoveArm,
            self.action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            "Move-arm action server ready:\n"
            f"  robot_id            = {self.robot_id}\n"
            f"  action              = {self.action_name}\n"
            f"  driver action       = {self.driver_action_name}\n"
            f"  driver wait timeout = {self.driver_wait_timeout_sec} s"
        )

    def _goal_callback(self, request: MoveArm.Goal) -> GoalResponse:
        if not math.isfinite(request.x) or not math.isfinite(request.z):
            self.get_logger().warning("Rejected arm goal: invalid coordinate.")
            return GoalResponse.REJECT

        with self._goal_lock:
            if self._goal_active:
                self.get_logger().warning(
                    "Rejected arm goal: another goal is already running."
                )
                return GoalResponse.REJECT
            self._goal_active = True

        self.get_logger().info(
            "Accepted arm goal: "
            f"x={request.x:.3f}m, z={request.z:.3f}m, relative={request.relative}."
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        del goal_handle
        self.get_logger().warning("Arm cancel request accepted.")
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> MoveArm.Result:
        result = MoveArm.Result()
        driver_goal_handle: Optional[Any] = None

        try:
            self.get_logger().info(
                f"Waiting for RoboMaster arm action {self.driver_action_name}."
            )
            if not self._driver_client.wait_for_server(
                timeout_sec=self.driver_wait_timeout_sec
            ):
                goal_handle.abort()
                result.success = False
                result.message = (
                    "RoboMaster arm action unavailable: "
                    f"{self.driver_action_name}"
                )
                self.get_logger().error(result.message)
                return result

            driver_goal = RobomasterMoveArm.Goal()
            driver_goal.x = float(goal_handle.request.x)
            driver_goal.z = float(goal_handle.request.z)
            driver_goal.relative = bool(goal_handle.request.relative)
            self.get_logger().info(
                "Forwarding arm goal to RoboMaster driver: "
                f"x={driver_goal.x:.3f}m, z={driver_goal.z:.3f}m, "
                f"relative={driver_goal.relative}."
            )

            response_event = Event()
            result_event = Event()
            state: Dict[str, Any] = {
                "goal_handle": None,
                "status": None,
                "error": None,
            }

            def feedback_callback(feedback_message) -> None:
                feedback = MoveArm.Feedback()
                feedback.progress = float(
                    max(0.0, min(1.0, feedback_message.feedback.progress))
                )
                goal_handle.publish_feedback(feedback)
                self.get_logger().info(
                    f"Arm movement progress: {feedback.progress * 100.0:.0f}%."
                )

            def driver_result_callback(future) -> None:
                try:
                    state["status"] = future.result().status
                    self.get_logger().info(
                        f"RoboMaster arm result received: status={state['status']}."
                    )
                except Exception as exception:
                    state["error"] = str(exception)
                finally:
                    result_event.set()

            def driver_goal_callback(future) -> None:
                try:
                    state["goal_handle"] = future.result()
                    if not state["goal_handle"].accepted:
                        state["error"] = "RoboMaster arm goal was rejected"
                        self.get_logger().error(state["error"])
                    else:
                        self.get_logger().info("RoboMaster driver accepted arm goal.")
                        state["goal_handle"].get_result_async().add_done_callback(
                            driver_result_callback
                        )
                except Exception as exception:
                    state["error"] = str(exception)
                finally:
                    response_event.set()

            send_future = self._driver_client.send_goal_async(
                driver_goal,
                feedback_callback=feedback_callback,
            )
            send_future.add_done_callback(driver_goal_callback)

            while rclpy.ok() and not response_event.wait(timeout=0.1):
                pass

            if not rclpy.ok():
                goal_handle.abort()
                result.success = False
                result.message = "ROS shutdown interrupted arm movement."
                return result

            if state["error"] is not None:
                goal_handle.abort()
                result.success = False
                result.message = str(state["error"])
                return result

            driver_goal_handle = state["goal_handle"]
            if goal_handle.is_cancel_requested:
                driver_goal_handle.cancel_goal_async()
                goal_handle.canceled()
                result.success = False
                result.message = "Arm movement canceled."
                return result

            while rclpy.ok() and not result_event.wait(timeout=0.1):
                if goal_handle.is_cancel_requested:
                    driver_goal_handle.cancel_goal_async()
                    goal_handle.canceled()
                    result.success = False
                    result.message = "Arm movement canceled."
                    return result

            if not rclpy.ok():
                if driver_goal_handle is not None:
                    driver_goal_handle.cancel_goal_async()
                goal_handle.abort()
                result.success = False
                result.message = "ROS shutdown interrupted arm movement."
                return result

            if state["error"] is not None:
                goal_handle.abort()
                result.success = False
                result.message = str(state["error"])
                return result

            if state["status"] == GoalStatus.STATUS_SUCCEEDED:
                goal_handle.succeed()
                result.success = True
                result.message = "Arm movement completed."
            elif state["status"] == GoalStatus.STATUS_CANCELED:
                goal_handle.canceled()
                result.success = False
                result.message = "RoboMaster arm movement was canceled."
            else:
                goal_handle.abort()
                result.success = False
                result.message = (
                    "RoboMaster arm movement failed with status "
                    f"{state['status']}."
                )
            if result.success:
                self.get_logger().info(result.message)
            else:
                self.get_logger().warning(result.message)
            return result

        except Exception as exception:
            if driver_goal_handle is not None:
                driver_goal_handle.cancel_goal_async()
            goal_handle.abort()
            result.success = False
            result.message = f"Arm movement exception: {exception}"
            self.get_logger().error(result.message)
            return result
        finally:
            with self._goal_lock:
                self._goal_active = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveArmServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
