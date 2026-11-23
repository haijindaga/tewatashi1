"""YASMIN states of the task.

Each state gets a shared TaskContext (see task_node.py) that gives access to
the arm (Motion), the operator approval gate and the status log.
"""

import threading

from sensor_msgs.msg import Image
from yasmin import State

from tewatashi_task.motion import MotionError

SUCCEEDED = "succeeded"
ABORTED = "aborted"
CANCELED = "canceled"
NO_CAMERA = "no_camera"
NOT_FOUND = "not_found"


class MoveToPose(State):
    """Plan a PTP motion to a named joint pose, wait for approval, execute."""

    def __init__(self, ctx, pose_name):
        super().__init__([SUCCEEDED, ABORTED, CANCELED])
        self._ctx = ctx
        self._pose_name = pose_name
        self.set_description(f"PTP motion to '{pose_name}'")

    def execute(self, blackboard):
        ctx = self._ctx
        try:
            joints = ctx.pose(self._pose_name)
            ctx.log(f"{self._pose_name} への軌道を計画中")
            trajectory = ctx.motion.plan_to_joints(joints)
            t = trajectory.points[-1].time_from_start
            seconds = t.sec + t.nanosec * 1e-9
            if self.is_canceled():
                return CANCELED
            if not ctx.wait_for_approval(
                self, f"{self._pose_name} へ移動（約 {seconds:.1f} 秒）。RViz の軌道を確認してください"
            ):
                return CANCELED
            ctx.log(f"{self._pose_name} へ移動中")
            if not ctx.motion.execute(trajectory, self.is_canceled):
                return CANCELED
            ctx.log(f"{self._pose_name} に到着")
            return SUCCEEDED
        except MotionError as exc:
            ctx.log(f"失敗: {exc}", error=True)
            return ABORTED

    def cancel_state(self):
        self._ctx.wake_approval()
        super().cancel_state()


class CaptureImage(State):
    """Wait for one color image from the wrist camera.

    The image is kept in memory only (no files). Without a camera (mock runs)
    the state reports no_camera after a timeout and the task goes on.
    """

    def __init__(self, ctx, topic, timeout_sec=3.0):
        super().__init__([SUCCEEDED, NO_CAMERA, CANCELED])
        self._ctx = ctx
        self._topic = topic
        self._timeout = timeout_sec
        self.set_description(f"wait for one image on {topic}")

    def execute(self, blackboard):
        ctx = self._ctx
        received = threading.Event()
        holder = {}

        def on_image(msg):
            holder["msg"] = msg
            received.set()

        sub = ctx.node.create_subscription(
            Image, self._topic, on_image, 1, callback_group=ctx.callback_group
        )
        try:
            ctx.log("画像を取得中")
            waited = 0.0
            while waited < self._timeout:
                if self.is_canceled():
                    return CANCELED
                if received.wait(0.1):
                    msg = holder["msg"]
                    ctx.set_latest_image(msg)
                    ctx.log(f"画像を取得 ({msg.width}x{msg.height})")
                    return SUCCEEDED
                waited += 0.1
            ctx.log(f"カメラ画像なし（{self._topic}）。この段階は飛ばします")
            return NO_CAMERA
        finally:
            ctx.node.destroy_subscription(sub)


class SegmentObject(State):
    """Ask the SAM3 service for a mask of the target object in the last image."""

    def __init__(self, ctx, timeout_sec=180.0):
        # The first call also loads the model, hence the long timeout.
        super().__init__([SUCCEEDED, NOT_FOUND, ABORTED, CANCELED])
        self._ctx = ctx
        self._timeout = timeout_sec
        self.set_description("SAM3: mask of the target object")

    def execute(self, blackboard):
        ctx = self._ctx
        image = ctx.latest_image()
        if image is None:
            ctx.log("切り出し: 画像がありません", error=True)
            return ABORTED
        target = ctx.target_object()
        client = ctx.segment_client()
        if not client.wait_for_service(timeout_sec=2.0):
            ctx.log("切り出し: sam3_server が起動していません", error=True)
            return ABORTED
        ctx.log(f"「{target}」を切り出し中（初回はモデル読み込みで時間がかかります）")
        request = ctx.segment_request(image, target)
        future = client.call_async(request)
        waited = 0.0
        while not future.done():
            if self.is_canceled():
                future.cancel()
                return CANCELED
            if waited > self._timeout:
                ctx.log("切り出し: 時間切れ", error=True)
                return ABORTED
            threading.Event().wait(0.1)
            waited += 0.1
        res = future.result()
        if res is None:
            ctx.log("切り出し: 応答なし", error=True)
            return ABORTED
        if not res.success:
            ctx.log(f"切り出し: {res.message}", error=("error" in res.message))
            return NOT_FOUND if "not found" in res.message else ABORTED
        ctx.set_latest_mask(res.mask, res.score)
        ctx.log(f"切り出し成功: {res.message}（パネルの「切り出し結果」に表示）")
        return SUCCEEDED
