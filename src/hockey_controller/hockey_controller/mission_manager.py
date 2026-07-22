#!/usr/bin/env python3

from threading import Event, Lock, Thread
from typing import Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from hockey_interfaces.action import NavigateToPoint, Spin


class MissionManager(Node):
    """Runs step1 navigation, then transitions to step2 spinning."""

    def __init__(self) -> None:
        super().__init__("mission_manager")

        self.declare_parameter("navigation_action", "navigate_to_point")
        self.declare_parameter("spin_action", "spin")
        self.declare_parameter("target_x", 1.0)
        self.declare_parameter("target_y", 0.0)
        self.declare_parameter("rotations", 1)
        self.declare_parameter("linear_speed", 0.4)
        self.declare_parameter("angular_speed", 0.8)
        self.declare_parameter("navigation_timeout_sec", 30.0)
        self.declare_parameter("spin_timeout_sec", 15.0)
        self.declare_parameter("action_wait_timeout_sec", 5.0)

        self.navigation_action = str(
            self.get_parameter("navigation_action").value
        )
        self.spin_action = str(self.get_parameter("spin_action").value)
        self.target_x = float(self.get_parameter("target_x").value)
        self.target_y = float(self.get_parameter("target_y").value)
        self.rotations = int(self.get_parameter("rotations").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.navigation_timeout_sec = float(
            self.get_parameter("navigation_timeout_sec").value
        )
        self.spin_timeout_sec = float(self.get_parameter("spin_timeout_sec").value)
        self.action_wait_timeout_sec = float(
            self.get_parameter("action_wait_timeout_sec").value
        )

        self._lock = Lock()
        self._running = False
        self._worker: Optional[Thread] = None
        self._callback_group = ReentrantCallbackGroup()

        self._status_publisher = self.create_publisher(
            String,
            "/mission/status",
            10,
        )
        self._start_service = self.create_service(
            Trigger,
            "/mission/start",
            self._handle_start,
            callback_group=self._callback_group,
        )
        self._navigation_client = ActionClient(
            self,
            NavigateToPoint,
            self.navigation_action,
            callback_group=self._callback_group,
        )
        self._spin_client = ActionClient(
            self,
            Spin,
            self.spin_action,
            callback_group=self._callback_group,
        )

        self._publish_status("IDLE")
        self.get_logger().info(
            "Mission manager ready. Call /mission/start.\n"
            f"  step1 action = {self.navigation_action}\n"
            f"  step2 action = {self.spin_action}\n"
            f"  target       = ({self.target_x:.2f}, {self.target_y:.2f})\n"
            f"  rotations    = {self.rotations}"
        )

    def _handle_start(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        with self._lock:
            if self._running:
                response.success = False
                response.message = "Mission is already running"
                return response

            self._running = True

        self._worker = Thread(target=self._run_mission, daemon=True)
        self._worker.start()

        response.success = True
        response.message = "Mission started"
        return response

    def _run_mission(self) -> None:
        try:
            self._publish_status("STEP1_NAVIGATE")
            navigation_success, navigation_message = self._run_navigation_step()
            if not navigation_success:
                self._publish_status("MISSION_FAILED")
                self.get_logger().error(
                    f"Step1 navigation failed: {navigation_message}"
                )
                return

            self.get_logger().info(
                "Step1 navigation succeeded. Transitioning to step2 spin."
            )

            self._publish_status("STEP2_SPIN")
            spin_success, spin_message = self._run_spin_step()
            if not spin_success:
                self._publish_status("MISSION_FAILED")
                self.get_logger().error(f"Step2 spin failed: {spin_message}")
                return

            self._publish_status("MISSION_DONE")
            self.get_logger().info("Mission completed successfully.")

        except Exception as exception:
            self._publish_status("MISSION_FAILED")
            self.get_logger().error(f"Mission exception: {exception}")

        finally:
            with self._lock:
                self._running = False

    def _run_navigation_step(self) -> Tuple[bool, str]:
        if not self._navigation_client.wait_for_server(
            timeout_sec=self.action_wait_timeout_sec
        ):
            return False, f"Action server unavailable: {self.navigation_action}"

        goal = NavigateToPoint.Goal()
        goal.target_x = self.target_x
        goal.target_y = self.target_y
        goal.linear_speed = self.linear_speed
        goal.angular_speed = self.angular_speed
        goal.timeout_sec = self.navigation_timeout_sec

        return self._send_goal_and_wait(
            self._navigation_client,
            goal,
            self._handle_navigation_feedback,
        )

    def _run_spin_step(self) -> Tuple[bool, str]:
        if not self._spin_client.wait_for_server(
            timeout_sec=self.action_wait_timeout_sec
        ):
            return False, f"Action server unavailable: {self.spin_action}"

        goal = Spin.Goal()
        goal.rotations = self.rotations
        goal.angular_speed = self.angular_speed
        goal.timeout_sec = self.spin_timeout_sec

        return self._send_goal_and_wait(
            self._spin_client,
            goal,
            self._handle_spin_feedback,
        )

    def _send_goal_and_wait(
        self,
        client,
        goal,
        feedback_callback,
    ) -> Tuple[bool, str]:
        done_event = Event()
        result_holder = {
            "success": False,
            "message": "Action did not finish",
        }

        def handle_goal_response(future) -> None:
            goal_handle = future.result()
            if not goal_handle.accepted:
                result_holder["message"] = "Goal rejected"
                done_event.set()
                return

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(handle_result)

        def handle_result(future) -> None:
            result_wrapper = future.result()
            result = result_wrapper.result
            result_holder["success"] = bool(result.success)
            result_holder["message"] = str(result.message)
            done_event.set()

        send_future = client.send_goal_async(
            goal,
            feedback_callback=feedback_callback,
        )
        send_future.add_done_callback(handle_goal_response)
        done_event.wait()

        return bool(result_holder["success"]), str(result_holder["message"])

    def _handle_navigation_feedback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.get_logger().info(
            "Step1 feedback: "
            f"{feedback.state}, "
            f"distance={feedback.distance_remaining:.2f}"
        )

    def _handle_spin_feedback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.get_logger().info(
            "Step2 feedback: "
            f"{feedback.state}, "
            f"rotation_remaining={feedback.rotation_remaining:.2f}"
        )

    def _publish_status(self, status: str) -> None:
        message = String()
        message.data = status
        self._status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManager()
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
