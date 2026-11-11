# Learning map: read main() first, then __init__(), then run().
# This client sends the tested motion; arm_controller executes the trajectory.
# Motion values and the home check are imported from sequence.py.
# No motion occurs just by importing this file.

"""Run the six-joint simulation demonstration once."""
# Python standard library. monotonic() measures elapsed real time.
# We use it for timeouts so pausing Gazebo cannot freeze our timeout clock.
import time

# rclpy is the ROS 2 Python library, similar to a communication SDK.
import rclpy
# ActionClient sends a task to an existing action server and receives its responses.
from rclpy.action import ActionClient
# Node is the base class providing ROS subscriptions, logging and other facilities.
from rclpy.node import Node
# QoS describes delivery behaviour; this profile can receive best-effort sensor data.
from rclpy.qos import qos_profile_sensor_data
# Controls who handles Ctrl+C, allowing our code to attempt cancellation first.
from rclpy.signals import SignalHandlerOptions
# Named action states, including STATUS_SUCCEEDED, instead of unexplained numbers.
from action_msgs.msg import GoalStatus
# A ROS duration message has seconds and nanoseconds fields.
from builtin_interfaces.msg import Duration
# Existing ROS action type containing Goal, Result and Feedback message definitions.
from control_msgs.action import FollowJointTrajectory
# Message describing the allowed tracking error for one named joint.
from control_msgs.msg import JointTolerance
# Message containing joint names and measured position, velocity and optional effort.
from sensor_msgs.msg import JointState
# One waypoint: joint coordinates, optional derivatives, and its arrival time.
from trajectory_msgs.msg import JointTrajectoryPoint

# These are OUR definitions from sequence.py, not ROS library functions.
# JOINTS gives the array order; WAYPOINTS gives (arrival time, six positions, stage label).
# home_error checks whether starting this particular experiment is appropriate.
from pieas_arm_control.sequence import JOINTS, WAYPOINTS, DURATION, home_error


# Our class inherits from Node, so its objects have ROS node capabilities.
# Read in execution order: main -> __init__ -> run -> make_goal.
# read_state and feedback are callbacks dispatched while spin_once runs.
# This is a motion-requesting program; the existing arm_controller executes it.
class JointSequence(Node):
    # Python automatically calls __init__ when main creates JointSequence().
    # self refers to this object, like this in C++; self.x persists between calls.
    def __init__(self):
        # Initialize the parent Node class and give this ROS node its graph name.
        super().__init__('pieas_joint_sequence')
        # Arguments: owning node, action type, and server endpoint name.
        # Creating this client does not send a goal or move the arm.
        self.client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        # Subscribe to encoder-like feedback from the simulated robot.
        # self.read_state passes the method itself, WITHOUT calling it now.
        # ROS calls it later with each received JointState message.
        self.subscription = self.create_subscription(JointState, '/joint_states', self.read_state, qos_profile_sensor_data)
        # None means no message received yet. Store the latest message here.
        self.state = None
        # Remember the previous simulation timestamp to detect advancing simulation.
        self.previous_stamp = None
        # Becomes true when a newly processed message has a later timestamp.
        self.state_advancing = False
        # Wall-clock arrival time lets us check whether feedback was recently received.
        self.received_at = 0.0
        # Wall-clock time of our last progress print; limits terminal output.
        self.last_print = 0.0
        self.last_stage = None
        # The accepted goal handle lets us request its result or cancel that specific goal.
        self.goal_handle = None
        # A future holds a response that will arrive later: here, acceptance or rejection.
        self.goal_future = None
        # A DIFFERENT future holds eventual execution completion, not mere acceptance.
        self.result_future = None

    # Subscription callback: message is supplied by ROS, not manually by us.
    # This stores measurements; it does not command the robot.
    def read_state(self, message):
        # A Python tuple (seconds, nanoseconds) can be compared in chronological order.
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        self.state_advancing = self.previous_stamp is not None and stamp > self.previous_stamp
        self.previous_stamp = stamp
        self.state = message
        self.received_at = time.monotonic()

    # Wait for an asynchronous response while continuing to process ROS events.
    # timeout is seconds of real elapsed time, independent of simulation speed.
    def wait_for(self, future, timeout):
        # Wall time keeps waits bounded even when Gazebo is paused.
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            # Process a ready ROS callback, or wait up to 0.1 seconds for work.
            # This allows incoming feedback and action responses to be handled.
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise TimeoutError('ROS request timed out. Check that Gazebo is running.')
        # Retrieve the completed response; an exception in the future propagates to main.
        return future.result()

    # Build the request in memory. This method does NOT send it.
    def make_goal(self):
        # Construct the Goal message belonging to the existing FollowJointTrajectory action.
        goal = FollowJointTrajectory.Goal()
        # Array values below correspond to names in exactly this order.
        goal.trajectory.joint_names = list(JOINTS)
        # Unpack a timestamp, full joint pose and human-readable stage label.
        # Arrival times are cumulative from trajectory start, not segment durations.
        for seconds, positions, label in WAYPOINTS:
            # Create a fresh object each iteration. append stores an object reference;
            # reusing one object would cause list entries to refer to that same object.
            point = JointTrajectoryPoint()
            # All six coordinates are supplied in the JOINTS order.
            # Joint 1 uses metres; rotary joints use radians. These are absolute coordinates.
            point.positions = list(positions)
            # Python creates a list of six zeros. These are ENDPOINT velocities:
            # stop at the waypoint, not stay still for the whole segment.
            point.velocities = [0.0] * 6
            # Assign the ROS duration; its omitted nanosec field defaults to zero.
            point.time_from_start = Duration(sec=seconds)
            # Add this waypoint to the goal. Repeated positions at later times create holds.
            goal.trajectory.points.append(point)
        # Explicit final-position tolerances: metres for joint 1, radians otherwise.
        # enumerate supplies both an index (0..5) and the corresponding joint name.
        for index, name in enumerate(JOINTS):
            # Allow 2 mm final position error for the slide and 0.01 rad for rotary joints.
            # The conditional expression chooses the tolerance using the joint index.
            goal.goal_tolerance.append(JointTolerance(name=name, position=0.002 if index == 0 else 0.01))
        # Permit up to three extra simulation seconds to meet the final goal criteria.
        # Abort if tracking error becomes large during the range test. This can
        # detect stalled tracking, but does not replace collision checking.
        for index, name in enumerate(JOINTS):
            goal.path_tolerance.append(JointTolerance(name=name, position=0.03 if index == 0 else 0.15))
        goal.goal_time_tolerance = Duration(sec=3)
        # The default zero trajectory header timestamp requests an immediate start
        # when the controller receives it. Return the message to run().
        return goal

    # Action feedback callback, separate from our /joint_states subscription.
    # It receives progress for the goal we sent, including desired and actual states.
    def feedback(self, message):
        feedback = message.feedback
        elapsed = (feedback.desired.time_from_start.sec
                   + feedback.desired.time_from_start.nanosec * 1e-9)
        # A waypoint label describes the movement leading TO that waypoint.
        stage = next((label for seconds, _, label in WAYPOINTS if elapsed < seconds),
                     'finishing')
        changed = stage != self.last_stage
        if changed:
            self.get_logger().info(f'Stage: {stage}')
            self.last_stage = stage
        # Print all joints every two real seconds, and immediately on stage changes.
        if not changed and time.monotonic() - self.last_print < 2.0:
            return
        self.last_print = time.monotonic()
        rows = []
        for number, name in enumerate(JOINTS):
            if name not in feedback.joint_names:
                continue
            index = feedback.joint_names.index(name)
            if min(len(feedback.desired.positions), len(feedback.actual.positions)) <= index:
                continue
            desired = feedback.desired.positions[index]
            actual = feedback.actual.positions[index]
            unit = 'm' if number == 0 else 'rad'
            rows.append(f'{name}: target={desired:.4f}, actual={actual:.4f}, '
                        f'error={desired - actual:.4f} {unit}')
        self.get_logger().info(f't={elapsed:.1f}/{DURATION}s | ' + ' | '.join(rows))

    # Retain the original goal-building lesson above for reading, but do not
    # run a sequence known to produce link penetration in the converted model.
    def run(self):
        raise RuntimeError(
            'The full-range sequence is retired after observed link penetration. '
            'Use pieas_arm_moveit_config plan_joint_goal for collision-checked planning.')

    # Cleanup path used after interruption or failure. Request cancellation of
    # our own goal only; cancellation is not a home movement or world reset.
    def cancel_pending(self):
        # A late goal acceptance must also be handled before exiting.
        # The request may have been sent but its acceptance not yet processed.
        if self.goal_handle is None and self.goal_future is not None:
            self.goal_handle = self.wait_for(self.goal_future, 3.0)
        if self.goal_handle is None or not self.goal_handle.accepted:
            return
        # A completed goal no longer needs a cancellation request.
        if self.result_future is not None and self.result_future.done():
            return
        # Send cancellation and wait briefly for the server to acknowledge it.
        response = self.wait_for(self.goal_handle.cancel_goal_async(), 3.0)
        # An empty list means the server did not confirm that a goal is being canceled.
        if not response.goals_canceling:
            raise RuntimeError('Controller did not acknowledge cancellation.')
        if self.result_future is None:
            self.result_future = self.goal_handle.get_result_async()
        # Also wait for a terminal result; acknowledgement alone is not completion.
        self.wait_for(self.result_future, 3.0)
        self.get_logger().info('Goal ended after cancellation request; verify the arm is stopped.')


# START READING HERE: setup.py registers this function for ros2 run.
# args=None lets rclpy use the command-line arguments, including ROS parameters.
def main(args=None):
    # Keep ROS available briefly after Ctrl+C so we can request cancellation.
    # Initialize ROS. Our exception handler below handles Ctrl+C before shutdown.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    # Create an object. Python now enters JointSequence.__init__ above.
    node = JointSequence()
    # Exit code zero conventionally means success; nonzero indicates failure.
    exit_code = 0
    # Run the experiment, with error/interruption handling and guaranteed cleanup.
    try:
        # Enter run() above. It waits, checks home, sends the goal and awaits completion.
        node.run()
    # Catch Ctrl+C or normal runtime errors instead of silently reporting success.
    except (KeyboardInterrupt, Exception) as error:
        exit_code = 1
        node.get_logger().error(str(error) or 'Interrupted by user.')
        try:
            # Attempt to stop an outstanding goal while ROS communication is still available.
            node.cancel_pending()
        except (KeyboardInterrupt, Exception) as cancel_error:
            node.get_logger().error(f'Cancellation not confirmed: {cancel_error}. Pause Gazebo to stop simulation motion.')
    # This block executes on success and on handled failure.
    finally:
        # Release this node. This does not shut down the separate Gazebo process.
        node.destroy_node()
        # Release the ROS context belonging to this Python process.
        rclpy.shutdown()
    return exit_code


# This block runs when executing this file directly with Python.
# ros2 run normally reaches main through the setup.py console entry point.
if __name__ == '__main__':
    # Run main and pass its return code back to the shell.
    raise SystemExit(main())
