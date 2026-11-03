import time
from pathlib import Path

from gz.msgs.image_pb2 import Image as GazeboImage
from gz.transport import Node
from PIL import Image as PILImage

TOPIC = (
    "/world/obstacle_course/model/x500_lidar_rgb_0/"
    "link/camera_link/sensor/IMX214/image"
)

OUTPUT = Path(
    "/home/hamngo/gazebo_ros2_drone_uni_group/"
    "drone-farm/captures/camera_snapshot.png"
)

node = Node()
received = False


def on_image(msg):
    global received

    if received:
        return

    image = PILImage.frombytes(
        "RGB",
        (msg.width, msg.height),
        msg.data,
    )

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    image.save(OUTPUT)

    received = True

    print(
        f"Saved camera_snapshot.png "
        f"({msg.width} x {msg.height})"
    )


if not node.subscribe(GazeboImage, TOPIC, on_image):
    raise RuntimeError("Could not subscribe to camera topic")

print("Waiting for camera frame...")

try:
    while not received:
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
