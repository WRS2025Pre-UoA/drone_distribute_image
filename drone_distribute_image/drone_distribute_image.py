import os
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from drone_distribute_image.gui_tool import Button, DrawTool


class DroneDistributeImage(Node):
    def __init__(self):
        super().__init__('drone_distribute_image')

        self.declare_parameter('target_dir', 'tmp_image/')
        self.target_dir = self.get_parameter(
            'target_dir').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Point,
            'gui_mouse_left',
            self.mouse_click_callback,
            10
        )

        self.bridge = CvBridge()

        # guiの準備
        self.gui_publisher = self.create_publisher(Image, "gui", 10)
        self.timer = self.create_timer(0.1, self.publish_gui)
        self.image_index = 0

        # 画面サイズ
        self.Width = 1280
        self.Height = 720+40

        # 各Publisher
        self.image_publishers = [
            self.create_publisher(Image, "pressure_image", 1),
            self.create_publisher(Image, "qr_image", 1),
            self.create_publisher(Image, "rust_image", 1),
            self.create_publisher(Image, "crack_image", 1),
            self.create_publisher(Image, "temperature_image", 1),
        ]
        self.send_publisher = self.create_publisher(String, "send_topic", 1)

        button_size = (150, 70)
        self.buttons = [
            Button((20*(1+i)+button_size[0]*i, 50), button_size) for i in range(5)
        ]
        self.send_button = Button((20*6+button_size[0]*5, 50), button_size)

        # 画像変更ボタン
        self.prev_button = Button((20, self.Height-70), (100, 50))
        self.next_button = Button((self.Width-120, self.Height-70), (100, 50))

    def publish_gui(self):
        # 画像一覧の取得
        images = [filename for filename in os.listdir(self.target_dir) if os.path.isfile(
            os.path.join(self.target_dir, filename)) and filename.endswith((".png", ".jpg", ".jpeg"))]
        images.sort()

        # 表示画像の選択
        shown_image = np.full((320, 640, 3), (255, 255, 0), dtype=np.uint8)
        shown_image_name = "None"
        if len(images) > 0:
            self.image_index = (self.image_index+len(images)) % len(images)

            shown_image_name = images[self.image_index]
            shown_image = cv2.imread(
                os.path.join(self.target_dir, shown_image_name))

        self.pub_image = shown_image.copy()

        # 描画
        tool = DrawTool(self.Width, self.Height, (127, 127, 127))

        # 画像の表示
        target_width = 1280
        target_height = 720
        h, w = shown_image.shape[:2]
        size = (w*target_height//h, target_height)
        if h*target_width <= target_height*w:
            size = (target_width, h*target_width//w)
        tool.draw_image(shown_image, (0, 40), size)

        # ファイル名の表示
        tool.draw_at_text(shown_image_name, (self.Width//2, 20),
                          size=1, color=(255, 255, 255), thickness=2)

        # ボタンの表示
        texts = ["Meter", "QR", "Rust", "Crack", "Temp"]
        background_color = (180, 180, 180)
        for btn, text in zip(self.buttons, texts):
            tool.draw_button(btn, text, color=background_color,
                             text_size=1.2, text_thickness=2)
        tool.draw_button(self.send_button, "Send",
                         color=background_color, text_size=1.2, text_thickness=2)
        tool.draw_button(self.prev_button, "Prev",
                         color=background_color, text_size=1.2, text_thickness=2)
        tool.draw_button(self.next_button, "Next",
                         color=background_color, text_size=1.2, text_thickness=2)

        try:
            ros_image = self.bridge.cv2_to_imgmsg(tool.image, "bgr8")
            self.gui_publisher.publish(ros_image)
        except CvBridgeError as e:
            self.get_logger().error(e)
            return

    def mouse_click_callback(self, msg):
        pos = (msg.x, msg.y)
        for i, btn in enumerate(self.buttons):
            if btn.is_clicked(pos):
                try:
                    ros_image = self.bridge.cv2_to_imgmsg(
                        self.pub_image, "bgr8")
                    self.image_publishers[i].publish(ros_image)
                except CvBridgeError as e:
                    self.get_logger().error(e)
                    return

        if self.send_button.is_clicked(pos):
            self.send_publisher.publish(String())

        if self.prev_button.is_clicked(pos):
            self.image_index -= 1
        if self.next_button.is_clicked(pos):
            self.image_index += 1


def main(args=None):
    rclpy.init(args=args)

    node = DroneDistributeImage()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
