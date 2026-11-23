"""SAM3 segmentation as a ROS 2 service.

  ~/segment      tewatashi_interfaces/Segment   image + object name -> mask
  ~/unload       std_srvs/Trigger               free the GPU memory
  ~/debug_image  sensor_msgs/Image              last result drawn over the image
                                                (re-sent at 1 Hz so late viewers see it)

Runs with the Python of the perception venv (see README): SAM3 is used through
Hugging Face transformers (Sam3Model, "facebook/sam3"). The model is loaded at
the first request unless `preload` is set, so the GPU stays free until needed.
"""

import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from tewatashi_interfaces.srv import Segment


def image_to_rgb(msg):
    """sensor_msgs/Image (rgb8 / bgr8 / rgba8 / bgra8) -> HxWx3 uint8 RGB."""
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(msg.encoding)
    if channels is None:
        raise ValueError(f"unsupported encoding '{msg.encoding}'")
    row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    img = row[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
    if msg.encoding.startswith("bgr"):
        img = img[:, :, [2, 1, 0]]
    else:
        img = img[:, :, :3]
    return np.ascontiguousarray(img)


def mono_to_image(mask, header):
    msg = Image(header=header, height=mask.shape[0], width=mask.shape[1],
                encoding="mono8", is_bigendian=0, step=mask.shape[1])
    msg.data = mask.tobytes()
    return msg


def rgb_to_image(rgb, header):
    msg = Image(header=header, height=rgb.shape[0], width=rgb.shape[1],
                encoding="rgb8", is_bigendian=0, step=rgb.shape[1] * 3)
    msg.data = rgb.tobytes()
    return msg


class Sam3Server(Node):
    def __init__(self):
        super().__init__("sam3_server")
        self.declare_parameter("model_id", "facebook/sam3")
        self.declare_parameter("device", "cuda")
        self.declare_parameter("dtype", "float32")
        self.declare_parameter("preload", False)
        self.declare_parameter("score_threshold", 0.5)
        self.declare_parameter("mask_threshold", 0.5)

        self._lock = threading.Lock()
        self._model = None
        self._processor = None
        group = MutuallyExclusiveCallbackGroup()
        self.create_service(Segment, "~/segment", self._on_segment, callback_group=group)
        self.create_service(Trigger, "~/unload", self._on_unload, callback_group=group)
        self._debug_pub = self.create_publisher(Image, "~/debug_image", 1)
        self._last_debug = None
        self.create_timer(1.0, self._republish_debug, callback_group=group)
        if self.get_parameter("preload").value:
            self._load()
        self.get_logger().info("ready (model is loaded at the first request)")

    def _load(self):
        if self._model is not None:
            return
        import torch
        from transformers import Sam3Model, Sam3Processor

        p = self.get_parameter
        dtype = {"float32": torch.float32, "bfloat16": torch.bfloat16,
                 "float16": torch.float16}[p("dtype").value]
        t0 = time.monotonic()
        self._processor = Sam3Processor.from_pretrained(p("model_id").value)
        self._model = Sam3Model.from_pretrained(p("model_id").value, dtype=dtype)
        self._model.to(p("device").value).eval()
        self.get_logger().info(f"SAM3 loaded in {time.monotonic() - t0:.1f} s ({p('dtype').value})")

    def _on_unload(self, _req, res):
        with self._lock:
            if self._model is not None:
                import torch
                self._model = None
                self._processor = None
                torch.cuda.empty_cache()
                res.message = "unloaded"
            else:
                res.message = "not loaded"
        res.success = True
        return res

    def _on_segment(self, req, res):
        import torch
        from PIL import Image as PILImage

        p = self.get_parameter
        with self._lock:
            try:
                self._load()
                rgb = image_to_rgb(req.image)
                inputs = self._processor(
                    images=PILImage.fromarray(rgb), text=req.prompt, return_tensors="pt"
                ).to(p("device").value)
                if self._model.dtype != torch.float32:
                    inputs["pixel_values"] = inputs["pixel_values"].to(self._model.dtype)
                with torch.no_grad():
                    outputs = self._model(**inputs)
                result = self._processor.post_process_instance_segmentation(
                    outputs,
                    threshold=p("score_threshold").value,
                    mask_threshold=p("mask_threshold").value,
                    target_sizes=inputs.get("original_sizes").tolist(),
                )[0]
            except Exception as exc:  # report to the caller instead of crashing the node
                res.success = False
                res.message = f"segmentation error: {exc}"
                self.get_logger().error(res.message)
                return res

        scores = [float(s) for s in result["scores"]]
        if not scores:
            res.success = False
            res.message = f"'{req.prompt}' not found"
            return res
        best = int(np.argmax(scores))
        mask = (result["masks"][best].cpu().numpy() > 0).astype(np.uint8) * 255
        res.success = True
        res.score = scores[best]
        res.message = f"'{req.prompt}' found ({len(scores)} candidates, best {res.score:.2f})"
        res.mask = mono_to_image(mask, req.image.header)
        self._publish_debug(rgb, mask, req.image.header)
        return res

    def _publish_debug(self, rgb, mask, header):
        overlay = rgb.copy()
        on = mask > 0
        overlay[on] = (0.5 * overlay[on] + 0.5 * np.array([255, 80, 0])).astype(np.uint8)
        self._last_debug = rgb_to_image(overlay, header)
        self._debug_pub.publish(self._last_debug)

    def _republish_debug(self):
        if self._last_debug is not None:
            self._debug_pub.publish(self._last_debug)


def main():
    rclpy.init()
    node = Sam3Server()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
