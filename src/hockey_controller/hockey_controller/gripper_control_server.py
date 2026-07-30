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
from robomaster_msgs.action import GripperControl as RobomasterGripperControl

from hockey_interfaces.action import GripperControl


class GripperControlServer(Node):
    """Forward project gripper goals to the RoboMaster gripper action."""

    def __init__(self) -> None:
        super().__init__("gripper_control_server")

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("action_name", "control_gripper")
        self.declare_parameter("driver_action_name", "gripper")
        self.declare_parameter("driver_wait_timeout_sec", 5.0)

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
            RobomasterGripperControl,
            self.driver_action_name,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            GripperControl,
            self.action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            "Gripper control action server ready:\n"
            f"  robot_id            = {self.robot_id}\n"
            f"  action              = {self.action_name}\n"
            f"  driver action       = {self.driver_action_name}\n"
            f"  driver wait timeout = {self.driver_wait_timeout_sec} s"
        )

    def _goal_callback(
        self,
        request: GripperControl.Goal,
    ) -> GoalResponse:
        valid_states = (
            GripperControl.Goal.PAUSE,
            GripperControl.Goal.OPEN,
            GripperControl.Goal.CLOSE,
        )
        if request.target_state not in valid_states:
            self.get_logger().warning("Rejected gripper goal: invalid state.")
            return GoalResponse.REJECT
        if not math.isfinite(request.power) or not 0.0 <= request.power <= 1.0:
            self.get_logger().warning(
                "Rejected gripper goal: power must be in [0, 1]."
            )
            return GoalResponse.REJECT

        with self._goal_lock:
            if self._goal_active:
                self.get_logger().warning(
                    "Rejected gripper goal: another goal is already running."
                )
                return GoalResponse.REJECT
            self._goal_active = True

        self.get_logger().info(
            "Accepted gripper goal: "
            f"target_state={request.target_state}, power={request.power:.2f}."
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        del goal_handle
        self.get_logger().warning("Gripper cancel request accepted.")
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> GripperControl.Result:
        result = GripperControl.Result()
        driver_goal_handle: Optional[Any] = None

        try:
            self.get_logger().info(
                "Waiting for RoboMaster gripper action "
                f"{self.driver_action_name}."
            )
            if not self._driver_client.wait_for_server(
                timeout_sec=self.driver_wait_timeout_sec
            ):
                goal_handle.abort()
                result.success = False
                result.message = (
                    "RoboMaster gripper action unavailable: "
                    f"{self.driver_action_name}"
                )
                self.get_logger().error(result.message)
                return result

            driver_goal = RobomasterGripperControl.Goal()
            driver_goal.target_state = int(goal_handle.request.target_state)
            driver_goal.power = float(goal_handle.request.power)
            self.get_logger().info(
                "Forwarding gripper goal to RoboMaster driver: "
                f"target_state={driver_goal.target_state}, "
                f"power={driver_goal.power:.2f}."
            )

            response_event = Event()
            result_event = Event()
            state: Dict[str, Any] = {
                "goal_handle": None,
                "status": None,
                "result": None,
                "error": None,
            }

            def feedback_callback(feedback_message) -> None:
                feedback = GripperControl.Feedback()
                feedback.current_state = int(
                    feedback_message.feedback.current_state
                )
                goal_handle.publish_feedback(feedback)
                self.get_logger().info(
                    "Gripper state feedback: "
                    f"current_state={feedback.current_state}."
                )

            def driver_result_callback(future) -> None:
                try:
                    wrapped_result = future.result()
                    state["status"] = wrapped_result.status
                    state["result"] = wrapped_result.result
                    self.get_logger().info(
                        "RoboMaster gripper result received: "
                        f"status={state['status']}."
                    )
                except Exception as exception:
                    state["error"] = str(exception)
                finally:
                    result_event.set()

            def driver_goal_callback(future) -> None:
                try:
                    state["goal_handle"] = future.result()
                    if not state["goal_handle"].accepted:
                        state["error"] = "RoboMaster gripper goal was rejected"
                        self.get_logger().error(state["error"])
                    else:
                        self.get_logger().info(
                            "RoboMaster driver accepted gripper goal."
                        )
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
                result.message = "ROS shutdown interrupted gripper control."
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
                result.message = "Gripper control canceled."
                return result

            while rclpy.ok() and not result_event.wait(timeout=0.1):
                if goal_handle.is_cancel_requested:
                    driver_goal_handle.cancel_goal_async()
                    goal_handle.canceled()
                    result.success = False
                    result.message = "Gripper control canceled."
                    return result

            if not rclpy.ok():
                driver_goal_handle.cancel_goal_async()
                goal_handle.abort()
                result.success = False
                result.message = "ROS shutdown interrupted gripper control."
                return result
            if state["error"] is not None:
                goal_handle.abort()
                result.success = False
                result.message = str(state["error"])
                return result

            driver_result = state["result"]
            if driver_result is not None:
                result.duration = driver_result.duration

            if state["status"] == GoalStatus.STATUS_SUCCEEDED:
                goal_handle.succeed()
                result.success = True
                result.message = "Gripper control completed."
            elif state["status"] == GoalStatus.STATUS_CANCELED:
                goal_handle.canceled()
                result.success = False
                result.message = "RoboMaster gripper control was canceled."
            else:
                goal_handle.abort()
                result.success = False
                result.message = (
                    "RoboMaster gripper control failed with status "
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
            result.message = f"Gripper control exception: {exception}"
            self.get_logger().error(result.message)
            return result
        finally:
            with self._goal_lock:
                self._goal_active = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperControlServer()
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
