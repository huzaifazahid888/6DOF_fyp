"""Send the previously tested 38-second joint-2 sequence once."""
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from pieas_arm_control.sequence import JOINTS, WAYPOINTS, home_error


class JointSequence(Node):
    def __init__(self):
        super().__init__('pieas_joint_sequence')
        self.client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.subscription = self.create_subscription(JointState, '/joint_states', self.read_state, qos_profile_sensor_data)
        self.state = None
        self.previous_stamp = None
        self.state_advancing = False
        self.received_at = 0.0
        self.last_print = 0.0
        self.goal_handle = None
        self.goal_future = None
        self.result_future = None

    def read_state(self, message):
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        self.state_advancing = self.previous_stamp is not None and stamp > self.previous_stamp
        self.previous_stamp = stamp
        self.state = message
        self.received_at = time.monotonic()

    def wait_for(self, future, timeout):
        # Wall time keeps waits bounded even when Gazebo is paused.
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise TimeoutError('ROS request timed out. Check that Gazebo is running.')
        return future.result()

    def make_goal(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for seconds, angle in WAYPOINTS:
            point = JointTrajectoryPoint()
            point.positions = [0.0, angle, 0.0, 0.0, 0.0, 0.0]
            point.velocities = [0.0] * 6
            point.time_from_start = Duration(sec=seconds)
            goal.trajectory.points.append(point)
        # Explicit final-position tolerances: metres for joint 1, radians otherwise.
        for index, name in enumerate(JOINTS):
            goal.goal_tolerance.append(JointTolerance(name=name, position=0.002 if index == 0 else 0.01))
        goal.goal_time_tolerance = Duration(sec=3)
        return goal

    def feedback(self, message):
        if time.monotonic() - self.last_print < 1.0:
            return
        self.last_print = time.monotonic()
        feedback = message.feedback
        if 'joint_2' not in feedback.joint_names:
            return
        index = feedback.joint_names.index('joint_2')
        if min(len(feedback.desired.positions), len(feedback.actual.positions)) <= index:
            return
        desired = feedback.desired.positions[index]
        actual = feedback.actual.positions[index]
        self.get_logger().info(f'joint_2: target={desired:.4f}, actual={actual:.4f}, error={desired - actual:.4f} rad')

    def run(self):
        self.get_logger().info('Waiting for the trajectory action server...')
        if not self.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('Action server unavailable. Start the lab and activate arm_controller.')
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.state_advancing and time.monotonic() - self.received_at < 1.0:
                break
        else:
            raise RuntimeError('No advancing joint feedback. Resume Gazebo and check joint_state_broadcaster.')
        reason = home_error(self.state.name, self.state.position, self.state.velocity)
        if reason:
            raise RuntimeError(reason + ' Return home before starting this lesson.')
        self.get_logger().info('Sending: +0.1 rad, hold, -0.1 rad, hold, home (38 simulation seconds).')
        self.goal_future = self.client.send_goal_async(self.make_goal(), feedback_callback=self.feedback)
        self.goal_handle = self.wait_for(self.goal_future, 10.0)
        if not self.goal_handle.accepted:
            raise RuntimeError('Controller rejected the trajectory.')
        self.get_logger().info('Goal accepted; waiting for execution to finish.')
        self.result_future = self.goal_handle.get_result_async()
        response = self.wait_for(self.result_future, 180.0)
        result = response.result
        if response.status != GoalStatus.STATUS_SUCCEEDED or result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f'Trajectory failed: status={response.status}, code={result.error_code}, {result.error_string}')
        self.get_logger().info('Trajectory succeeded. The final waypoint is home.')

    def cancel_pending(self):
        # A late goal acceptance must also be handled before exiting.
        if self.goal_handle is None and self.goal_future is not None:
            self.goal_handle = self.wait_for(self.goal_future, 3.0)
        if self.goal_handle is None or not self.goal_handle.accepted:
            return
        if self.result_future is not None and self.result_future.done():
            return
        response = self.wait_for(self.goal_handle.cancel_goal_async(), 3.0)
        if not response.goals_canceling:
            raise RuntimeError('Controller did not acknowledge cancellation.')
        if self.result_future is None:
            self.result_future = self.goal_handle.get_result_async()
        self.wait_for(self.result_future, 3.0)
        self.get_logger().info('Goal ended after cancellation request; verify the arm is stopped.')


def main(args=None):
    # Keep ROS available briefly after Ctrl+C so we can request cancellation.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = JointSequence()
    exit_code = 0
    try:
        node.run()
    except (KeyboardInterrupt, Exception) as error:
        exit_code = 1
        node.get_logger().error(str(error) or 'Interrupted by user.')
        try:
            node.cancel_pending()
        except (KeyboardInterrupt, Exception) as cancel_error:
            node.get_logger().error(f'Cancellation not confirmed: {cancel_error}. Pause Gazebo to stop simulation motion.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
