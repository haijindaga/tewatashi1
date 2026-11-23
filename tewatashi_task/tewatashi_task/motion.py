"""Arm motion through MoveIt only, with fixed safety settings.

Every trajectory goes through move_group (never straight to the controller),
because the ros2_control trajectory controller does not check joint limits.

Planner: Pilz PTP (deterministic; the plan fails instead of moving when a
collision is found on the path).

Scaling is always set explicitly. move_group treats a scaling factor of 0.0
(the default in a raw MotionPlanRequest, and in pymoveit2) as 1.0, i.e. full
speed (time_optimal_trajectory_generation.cpp, MoveIt 2 Humble). The 0.1
default in joint_limits.yaml is only applied by RViz / MoveGroupInterface.
"""

import threading
import time

from moveit_msgs.msg import DisplayTrajectory, RobotState, RobotTrajectory
from pymoveit2 import MoveIt2, MoveIt2State
from sensor_msgs.msg import JointState

UR_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Project limits on top of MoveIt's own checks.
MAX_SCALING = 0.3
JOINT_LIMIT_RAD = 3.14159


class MotionError(RuntimeError):
    pass


class Motion:
    def __init__(self, node, callback_group, velocity_scaling, acceleration_scaling):
        for name, value in (("velocity", velocity_scaling), ("acceleration", acceleration_scaling)):
            if not 0.0 < value <= MAX_SCALING:
                raise MotionError(
                    f"{name}_scaling={value} is outside (0, {MAX_SCALING}]. Refusing to start."
                )
        self._node = node
        self._moveit2 = MoveIt2(
            node=node,
            joint_names=UR_JOINTS,
            base_link_name="base_link",
            end_effector_name="tool0",
            group_name="ur_arm",
            callback_group=callback_group,
        )
        self._moveit2.pipeline_id = "pilz_industrial_motion_planner"
        self._moveit2.planner_id = "PTP"
        self._moveit2.max_velocity = velocity_scaling
        self._moveit2.max_acceleration = acceleration_scaling
        self._moveit2.allowed_planning_time = 5.0
        self._moveit2.num_planning_attempts = 1

        self._display_pub = node.create_publisher(DisplayTrajectory, "/display_planned_path", 1)
        self._lock = threading.Lock()
        self._last_joint_state = None
        node.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10, callback_group=callback_group
        )

    def _on_joint_state(self, msg):
        with self._lock:
            self._last_joint_state = msg

    def current_joint_state(self):
        with self._lock:
            return self._last_joint_state

    def plan_to_joints(self, joint_positions):
        """Plan a PTP motion. Returns a JointTrajectory or raises MotionError."""
        if len(joint_positions) != len(UR_JOINTS):
            raise MotionError(f"expected {len(UR_JOINTS)} joint values, got {len(joint_positions)}")
        for name, value in zip(UR_JOINTS, joint_positions):
            if abs(value) > JOINT_LIMIT_RAD:
                raise MotionError(f"{name}={value:.3f} rad is outside +-{JOINT_LIMIT_RAD} rad")
        if self.current_joint_state() is None:
            raise MotionError("no /joint_states received yet (is the driver running?)")

        trajectory = self._moveit2.plan(
            joint_positions=list(joint_positions), joint_names=UR_JOINTS, timeout_sec=15.0
        )
        if trajectory is None or not trajectory.points:
            raise MotionError(
                f"planning failed ({self._moveit2.last_failure() or 'no trajectory'})"
            )
        self._show(trajectory)
        return trajectory

    def _show(self, trajectory):
        """Draw the plan in RViz (MotionPlanning > Planned Path)."""
        msg = DisplayTrajectory()
        start = RobotState()
        start.joint_state = self.current_joint_state()
        msg.trajectory_start = start
        msg.trajectory = [RobotTrajectory(joint_trajectory=trajectory)]
        self._display_pub.publish(msg)

    def execute(self, trajectory, is_canceled):
        """Execute a planned trajectory and block until it ends.

        Returns True on success, False if canceled. Raises MotionError on failure.
        """
        if not self._moveit2.execute(trajectory):
            raise MotionError("execution was not accepted by move_group")
        duration = trajectory.points[-1].time_from_start
        deadline = time.monotonic() + duration.sec + duration.nanosec * 1e-9 + 10.0
        while time.monotonic() < deadline:
            if is_canceled():
                self.stop()
                return False
            if self._moveit2.query_state() == MoveIt2State.IDLE:
                # Finished: returns the stored outcome of the last motion.
                if self._moveit2.wait_until_executed(timeout_sec=0.0):
                    return True
                error = self._moveit2.get_last_execution_error_code()
                raise MotionError(
                    f"execution failed (MoveIt error code {getattr(error, 'val', '?')})"
                )
            time.sleep(0.05)
        self.stop()
        raise MotionError("execution took far longer than planned; stopped")

    def stop(self):
        """Ask move_group to stop the running trajectory (software stop)."""
        self._moveit2.cancel_execution()
        self._moveit2.stop_all_trajectory_execution()
