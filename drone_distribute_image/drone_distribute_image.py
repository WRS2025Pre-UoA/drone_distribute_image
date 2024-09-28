import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


# 分配器でキー入力で画像を送る
class DroneImage(Node):
    def __init__(self):
        super().__init__('image_receiver',namespace="tree")
        self.subscription = self.create_subscription(
            Image,
            'input_image_topic',
            self.image_callback,
            10
        )
        self.subscription = self.create_subscription(
            Point,
            'gui_mouse_left',
            self.mouse_click_callback,
            10
        )
        self.image = None
        self.cv_image = None  # OpenCV形式の画像（表示用）
        self.get_logger().info("Image Receiver Node Initialized")
        self.bridge = CvBridge()
        self.point=None

    def image_callback(self, msg):
        self.image = msg
        # ROS2のImageメッセージをOpenCV形式に変換
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # self.get_logger().info("Image received")
    
    def mouse_click_callback(self, msg):
        self.point = msg
        # ROS2のImageメッセージをOpenCV形式に変換
        self.get_logger().info("Point received")

    def get_cv_image(self):
        return self.cv_image

    def get_ros_image(self):
        return self.image

class ButtonHandler:
    def __init__(self, image_sender):
        self.image_sender = image_sender
        # ボタンの位置とサイズ
        self.button_positions = [(10, 70), (130, 70), (10, 140), (130, 140), (10, 210), (130, 210)]
        self.button_width = 110
        self.button_height = 50
        # 各ボタンに対応するテキスト
        self.texts = ["Meter", "QR", "Rust", "Crack", "Temp", "Send"]

    def draw_buttons(self, img):
        # 背景に黒の長方形を描画 (250x280)
        # background_top_left = (0, 0)
        # background_bottom_right = (250, 280)
        # cv2.rectangle(img, background_top_left, background_bottom_right, (0, 0, 0), -1)

        # 各ボタンを描画
        for i, pos in enumerate(self.button_positions):
            top_left = pos
            bottom_right = (top_left[0] + self.button_width, top_left[1] + self.button_height)
            # 白でボタンを描画
            cv2.rectangle(img, top_left, bottom_right, (255, 255, 255), -1)
            # ボタン上にテキストを描画
            cv2.putText(img, self.texts[i], (top_left[0] + 10, top_left[1] + 35), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            for i, pos in enumerate(self.button_positions):
                top_left = pos
                bottom_right = (top_left[0] + self.button_width, top_left[1] + self.button_height)
                if top_left[0] <= x <= bottom_right[0] and top_left[1] <= y <= bottom_right[1]:
                    print(f"Button {i+1} clicked, sending image to {self.texts[i]}")
                    # ボタンに対応するトピックに画像を送信
                    self.image_sender.send_image_to_topic(i + 1)
    
    def clicked_image(self, x, y):
        for i, pos in enumerate(self.button_positions):
            top_left = pos
            bottom_right = (top_left[0] + self.button_width, top_left[1] + self.button_height)
            if top_left[0] <= x <= bottom_right[0] and top_left[1] <= y <= bottom_right[1]:
                print(f"Button {i+1} clicked, sending image to {self.texts[i]}")
                # ボタンに対応するトピックに画像を送信
                self.image_sender.send_image_to_topic(i + 1)

class ImageSender(Node):
    def __init__(self):
        super().__init__('image_sender',namespace="tree")
        self.publisher1 = self.create_publisher(Image, 'pressure_image', 1)
        self.publisher2 = self.create_publisher(Image, 'qr_image', 1)
        self.publisher3 = self.create_publisher(Image, 'rust_image', 1)
        self.publisher4 = self.create_publisher(Image, 'crack_image', 1)
        self.publisher5 = self.create_publisher(Image, 'temperature_image', 1)
        self.publisher6 = self.create_publisher(Image, 'send_image', 1)
        self.gui_publish = self.create_publisher(Image, 'gui', 1)
        self.last_received_image = None

    def update_last_received_image(self, image):
        self.last_received_image = image

    def send_image_to_topic(self, topic_number):
        if self.last_received_image is not None:
            if topic_number == 1:
                self.publisher1.publish(self.last_received_image)
            elif topic_number == 2:
                self.publisher2.publish(self.last_received_image)
            elif topic_number == 3:
                self.publisher3.publish(self.last_received_image)
            elif topic_number == 4:
                self.publisher4.publish(self.last_received_image)
            elif topic_number == 5:
                self.publisher5.publish(self.last_received_image)
            elif topic_number == 6:
                self.publisher6.publish(self.last_received_image)
            self.get_logger().info(f"Image sent to topic{topic_number}")
        else:
            self.get_logger().info("No image received to send.")

def main(args=None):
    rclpy.init(args=args)
    
    image_receiver = ImageReceiver()
    image_sender = ImageSender()
    button_handler = ButtonHandler(image_sender)

    bridge = CvBridge()

    # cv2.namedWindow("Image with Buttons")
    # cv2.setMouseCallback("Image with Buttons", button_handler.on_mouse_click)

    while rclpy.ok():
        rclpy.spin_once(image_receiver,timeout_sec=1)

        cv_image = image_receiver.get_cv_image()
        ros_image = image_receiver.get_ros_image()

        if cv_image is not None:
             # 黒い長方形の背景画像を作成
            img_with_buttons = np.zeros((280, 250, 3), dtype=np.uint8)
            # テキスト「MISORA 2」を高さ70以内に中央に描画
            text = "MISORA 2"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1.5
            font_thickness = 2
            text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
            text_x = (img_with_buttons.shape[1] - text_size[0]) // 2  # 中央に配置
            text_y = 50  # 高さ70以内の中央
            # 黒い背景に白で文字を描画
            cv2.putText(img_with_buttons, text, (text_x, text_y), font, font_scale, (255, 255, 255), font_thickness)
            button_handler.draw_buttons(img_with_buttons)

            # OpenCVで画像を表示
            # cv2.imshow("Image with Buttons", img_with_buttons)
            # cv2.waitKey(1)  # OpenCVの画面を更新
            try:
                selection_image = bridge.cv2_to_imgmsg(img_with_buttons, 'bgr8')
                image_sender.gui_publish.publish(selection_image)
            except CvBridgeError as e:
                print(e)
            
            if image_receiver.point != None:
                button_handler.clicked_image(image_receiver.point.x, image_receiver.point.y)
                image_receiver.point=None

            # 最新の画像をImageSenderに渡す
            image_sender.update_last_received_image(ros_image)

    image_receiver.destroy_node()
    image_sender.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
