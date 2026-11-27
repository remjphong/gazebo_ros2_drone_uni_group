import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# Topic camera thật của drone trong Gazebo
CAMERA_TOPIC = (
    "/world/plant_test/model/x500_mono_cam_0/"
    "link/camera_link/sensor/camera/image"
)


class DiseaseDetection(Node):

    def __init__(self):
        super().__init__("drone_disease_detection")

        self.bridge = CvBridge()
        self.last_warning = False

        self.subscription = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_callback,
            10
        )

        self.get_logger().info("Da ket noi camera Gazebo!")
        self.get_logger().info(f"Camera topic: {CAMERA_TOPIC}")

    def image_callback(self, msg):

        try:
            # Chuyen anh ROS 2 thanh anh OpenCV
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )

        except Exception as error:
            self.get_logger().error(f"Loi doc anh: {error}")
            return

        # Chuyen sang khong gian mau HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Nguong mau vang
        lower_yellow = np.array([15, 100, 100])
        upper_yellow = np.array([35, 255, 255])

        # Nguong mau do/cam
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([15, 255, 255])

        mask_yellow = cv2.inRange(
            hsv, lower_yellow, upper_yellow
        )

        mask_red = cv2.inRange(
            hsv, lower_red, upper_red
        )

        mask = cv2.bitwise_or(mask_yellow, mask_red)

        # Tinh ty le pixel co mau nghi ngo
        disease_ratio = (
            np.count_nonzero(mask) / mask.size
        )

        # Nguong canh bao 0.5%
        detected = disease_ratio > 0.005

        if detected:
            cv2.putText(
                frame,
                "WARNING: SUSPECTED COLOR",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

        cv2.putText(
            frame,
            f"Color ratio: {disease_ratio:.2%}",
            (20, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        # Chi in khi trang thai canh bao thay doi
        if detected and not self.last_warning:
            self.get_logger().warning(
                f"Phat hien mau nghi ngo! "
                f"Ty le: {disease_ratio:.2%}"
            )

        self.last_warning = detected

        # Hien thi camera va mat na mau
        cv2.imshow("Drone Camera - Gazebo", frame)
        cv2.imshow("Disease Detection Mask", mask)


def main(args=None):

    rclpy.init(args=args)

    node = DiseaseDetection()

    print("DANG NHAN HINH ANH TU CAMERA DRONE GAZEBO")
    print("Nhan phim q tren cua so camera de thoat.")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
