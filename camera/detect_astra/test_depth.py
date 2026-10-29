import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2


class DepthReader(Node):
    def __init__(self):
        super().__init__('depth_reader')

        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.u = 320
        self.v = 240

        cv2.namedWindow("Astra Pro Depth")
        cv2.setMouseCallback("Astra Pro Depth", self.mouse_callback)

        print("Dang mo camera Depth...")
        print("Click chuot de chon diem do")
        print("Nhan C de ve tam anh, Q de thoat")

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.u = x
            self.v = y

    def depth_callback(self, msg):
        if msg.encoding != '16UC1':
            return

        depth = np.frombuffer(
            msg.data, dtype=np.uint16
        ).reshape(msg.height, msg.step // 2)[:, :msg.width]

        # Tạo hình ảnh màu từ Depth
        depth_m = depth.astype(np.float32) / 1000.0

        depth_display = np.clip(
            depth_m / 5.0 * 255, 0, 255
        ).astype(np.uint8)

        depth_color = cv2.applyColorMap(
            depth_display, cv2.COLORMAP_JET
        )

        depth_color[depth == 0] = (0, 0, 0)

        # Lấy điểm đang đo
        u = max(0, min(self.u, msg.width - 1))
        v = max(0, min(self.v, msg.height - 1))

        distance_mm = int(depth[v, u])

        # Vẽ dấu cộng tại điểm đo
        cv2.drawMarker(
            depth_color,
            (u, v),
            (0, 0, 255),
            cv2.MARKER_CROSS,
            25,
            2
        )

        # Hiển thị tọa độ và khoảng cách
        if distance_mm > 0:
            text = (
                f"Pixel ({u},{v}) | "
                f"Distance: {distance_mm / 1000:.3f} m"
            )
        else:
            text = f"Pixel ({u},{v}) | No valid depth"

        cv2.rectangle(
            depth_color,
            (0, 0),
            (msg.width, 35),
            (0, 0, 0),
            -1
        )

        cv2.putText(
            depth_color,
            text,
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )

        cv2.imshow("Astra Pro Depth", depth_color)


def main():
    rclpy.init()
    node = DepthReader()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('c'):
                node.u = 320
                node.v = 240

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()