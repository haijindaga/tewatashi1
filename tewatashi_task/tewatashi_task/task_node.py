"""Resident task node.

Started once (from tewatashi_bringup) and then driven by the operator:

  ~/start   std_srvs/Trigger  run the task state machine from the beginning
  ~/stop    std_srvs/Trigger  cancel the running state machine and stop the arm
  ~/home    std_srvs/Trigger  move to the 'home' pose (only when idle)
  ~/approve std_srvs/Trigger  approve the motion that is waiting
  ~/set_require_approval std_srvs/SetBool
                              ask before every motion (cannot be turned off on
                              real hardware)
  ~/status  std_msgs/String   JSON status for the operator panel (latched)

The Trigger-service style follows the UR driver's dashboard client
(/dashboard_client/play, /stop, ...). State machines are YASMIN and are shown
in yasmin_viewer.
"""

import collections
import json
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from yasmin import Blackboard, StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub

from tewatashi_task.motion import Motion
from tewatashi_interfaces.srv import Segment
from tewatashi_task.states import (
    ABORTED, CANCELED, NO_CAMERA, NOT_FOUND, SUCCEEDED, CaptureImage, MoveToPose, SegmentObject,
)

DONE = "done"
FAILED = "failed"


class TaskContext:
    """What the states may use. Keeps the states free of node plumbing."""

    def __init__(self, owner):
        self._owner = owner
        self.node = owner
        self.callback_group = owner.motion_group
        self.motion = owner.motion
        self._approve = threading.Event()

    def pose(self, name):
        return self._owner.poses[name]

    def log(self, text, error=False):
        self._owner.add_log(text, error)

    def set_latest_image(self, msg):
        self._owner.latest_image = msg

    def latest_image(self):
        return self._owner.latest_image

    def set_latest_mask(self, mask, score):
        self._owner.latest_mask = mask
        self._owner.latest_mask_score = score

    def target_object(self):
        # Read at every run, so `ros2 param set /tewatashi_task target_object ...`
        # takes effect without a restart.
        return self._owner.get_parameter("target_object").value

    def segment_client(self):
        return self._owner.segment_client

    @staticmethod
    def segment_request(image, prompt):
        return Segment.Request(image=image, prompt=prompt)

    def wait_for_approval(self, state, text):
        """Block until the operator approves. False if the state was canceled."""
        if not self._owner.require_approval:
            return True
        self._approve.clear()
        self._owner.set_waiting(text)
        try:
            while not self._approve.wait(0.1):
                if state.is_canceled():
                    return False
            return not state.is_canceled()
        finally:
            self._owner.set_waiting(None)

    def approve(self):
        self._approve.set()

    def wake_approval(self):
        self._approve.set()


class TaskNode(Node):
    def __init__(self):
        super().__init__("tewatashi_task")
        set_ros_loggers(self)
        self.declare_parameter("use_fake_hardware", True)
        self.declare_parameter("velocity_scaling", 0.1)
        self.declare_parameter("acceleration_scaling", 0.1)
        self.declare_parameter("require_approval", True)
        self.declare_parameter("home_on_startup", True)
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("target_object", "hammer")
        self.declare_parameter("pose_names", ["home", "observe"])
        self.declare_parameter("pose.home", [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0])
        self.declare_parameter("pose.observe", [0.0, -1.9, 1.6, -1.27, -1.57, 0.0])

        p = self.get_parameter
        self.fake = p("use_fake_hardware").value
        self.require_approval = bool(p("require_approval").value)
        if not self.fake and not self.require_approval:
            self.get_logger().warn("require_approval is forced to true on real hardware")
            self.require_approval = True
        self.poses = {n: list(p(f"pose.{n}").value) for n in p("pose_names").value}

        self.motion_group = ReentrantCallbackGroup()
        self.motion = Motion(
            self, self.motion_group, p("velocity_scaling").value, p("acceleration_scaling").value
        )
        self.latest_image = None
        self.latest_mask = None
        self.latest_mask_score = 0.0
        self.segment_client = self.create_client(
            Segment, "/sam3_server/segment", callback_group=self.motion_group
        )

        self._lock = threading.Lock()
        self._log = collections.deque(maxlen=30)
        self._waiting_text = None
        self._running_sm = None
        self._running_name = None
        self._last_result = "-"
        self._worker = None

        self.ctx = TaskContext(self)
        image_topic = p("image_topic").value
        self.task_sm = self._build_task_sm(image_topic)
        self.home_sm = self._build_home_sm()
        self._viewers = [
            YasminViewerPub(self.task_sm, "TASK", node=self),
            YasminViewerPub(self.home_sm, "HOME", node=self),
        ]

        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(String, "~/status", status_qos)
        srv_group = MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger, "~/start", self._on_start, callback_group=srv_group)
        self.create_service(Trigger, "~/stop", self._on_stop, callback_group=srv_group)
        self.create_service(Trigger, "~/home", self._on_home, callback_group=srv_group)
        self.create_service(Trigger, "~/approve", self._on_approve, callback_group=srv_group)
        self.create_service(
            SetBool, "~/set_require_approval", self._on_set_approval, callback_group=srv_group
        )
        self.create_timer(0.5, self.publish_status, callback_group=srv_group)
        # Only used to see whether move_group is up before the start-up homing.
        self._plan_client = self.create_client(GetMotionPlan, "plan_kinematic_path")

        mode = "模擬ハードウェア" if self.fake else "実機"
        self.add_log(f"起動しました（{mode}、速度 {p('velocity_scaling').value:.0%}）")
        if p("home_on_startup").value:
            self._startup_timer = self.create_timer(
                2.0, self._startup_home, callback_group=srv_group
            )

    # ---------------------------------------------------------------- machines
    def _build_task_sm(self, image_topic):
        sm = StateMachine(outcomes=[DONE, FAILED, CANCELED])
        sm.add_state(
            "GO_HOME", MoveToPose(self.ctx, "home"),
            transitions={SUCCEEDED: "GO_OBSERVE", ABORTED: FAILED, CANCELED: CANCELED},
        )
        sm.add_state(
            "GO_OBSERVE", MoveToPose(self.ctx, "observe"),
            transitions={SUCCEEDED: "CAPTURE", ABORTED: FAILED, CANCELED: CANCELED},
        )
        sm.add_state(
            "CAPTURE", CaptureImage(self.ctx, image_topic),
            transitions={SUCCEEDED: "SEGMENT", NO_CAMERA: "RETURN_HOME", CANCELED: CANCELED},
        )
        # Part estimation, grasp candidates, decision (carry directly / place
        # and regrasp) and handover states will be inserted after SEGMENT.
        sm.add_state(
            "SEGMENT", SegmentObject(self.ctx),
            transitions={SUCCEEDED: "RETURN_HOME", NOT_FOUND: "RETURN_HOME",
                         ABORTED: "RETURN_HOME", CANCELED: CANCELED},
        )
        sm.add_state(
            "RETURN_HOME", MoveToPose(self.ctx, "home"),
            transitions={SUCCEEDED: DONE, ABORTED: FAILED, CANCELED: CANCELED},
        )
        sm.set_sigint_handler(False)
        return sm

    def _build_home_sm(self):
        sm = StateMachine(outcomes=[DONE, FAILED, CANCELED])
        sm.add_state(
            "GO_HOME", MoveToPose(self.ctx, "home"),
            transitions={SUCCEEDED: DONE, ABORTED: FAILED, CANCELED: CANCELED},
        )
        sm.set_sigint_handler(False)
        return sm

    # ---------------------------------------------------------------- running
    def _run(self, name, sm):
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                return False, f"実行中です（{self._running_name}）。先に停止してください"
            self._running_sm = sm
            self._running_name = name
            self._worker = threading.Thread(target=self._worker_main, args=(name, sm), daemon=True)
            self._worker.start()
        return True, f"{name} を開始しました"

    def _worker_main(self, name, sm):
        self.add_log(f"[{name}] 開始")
        try:
            outcome = sm(Blackboard())
        except Exception as exc:  # YASMIN raises when canceled as a whole
            outcome = CANCELED if sm.is_canceled() else FAILED
            if outcome == FAILED:
                self.add_log(f"[{name}] 例外: {exc}", error=True)
        result = {DONE: "完了", FAILED: "失敗", CANCELED: "停止"}.get(outcome, outcome)
        with self._lock:
            self._running_sm = None
            self._running_name = None
            self._last_result = f"{name}: {result}"
        self.add_log(f"[{name}] {result}", error=(outcome == FAILED))

    def _startup_home(self):
        self.destroy_timer(self._startup_timer)
        waiting_for = []
        if self.motion.current_joint_state() is None:
            waiting_for.append("joint_states")
        if not self._plan_client.service_is_ready():
            waiting_for.append("move_group")
        if waiting_for:
            self.add_log(f"{' と '.join(waiting_for)} の起動待ち（初期位置への移動は後で）")
            self._startup_timer = self.create_timer(3.0, self._startup_home)
            return
        self._run("HOME", self.home_sm)

    # ---------------------------------------------------------------- services
    def _on_start(self, _req, res):
        res.success, res.message = self._run("TASK", self.task_sm)
        return res

    def _on_home(self, _req, res):
        res.success, res.message = self._run("HOME", self.home_sm)
        return res

    def _on_stop(self, _req, res):
        with self._lock:
            sm = self._running_sm
        self.motion.stop()
        if sm is not None:
            sm.cancel_state_machine()
            self.add_log("停止しました。やり直すときは「初期位置へ」→「スタート」")
            res.message = "停止しました"
        else:
            res.message = "実行中のものはありません（念のため停止指令は送りました）"
        res.success = True
        return res

    def _on_approve(self, _req, res):
        if self._waiting_text is None:
            res.success, res.message = False, "承認待ちではありません"
        else:
            self.ctx.approve()
            res.success, res.message = True, "承認しました"
        return res

    def _on_set_approval(self, req, res):
        if not req.data and not self.fake:
            res.success, res.message = False, "実機では確認を外せません"
            return res
        self.require_approval = bool(req.data)
        self.add_log("動作前の確認: " + ("あり" if req.data else "なし（模擬のみ）"))
        res.success, res.message = True, "変更しました"
        return res

    # ---------------------------------------------------------------- status
    def add_log(self, text, error=False):
        stamp = time.strftime("%H:%M:%S")
        with self._lock:
            self._log.append({"t": stamp, "text": text, "error": error})
        (self.get_logger().error if error else self.get_logger().info)(text)
        self.publish_status()

    def set_waiting(self, text):
        with self._lock:
            self._waiting_text = text
        self.publish_status()

    def publish_status(self):
        with self._lock:
            sm = self._running_sm
            status = {
                "mode": "mock" if self.fake else "real",
                "busy": sm is not None,
                "task": self._running_name,
                "state": sm.get_current_state() if sm is not None else "",
                "waiting_approval": self._waiting_text is not None,
                "approval_text": self._waiting_text or "",
                "require_approval": self.require_approval,
                "last_result": self._last_result,
                "log": list(self._log),
            }
        self._status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))


def main():
    rclpy.init()
    node = TaskNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.motion.stop()
        with node._lock:
            sm = node._running_sm
        if sm is not None:
            sm.cancel_state_machine()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
