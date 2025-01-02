import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        # 订阅 /camera/image/compressed 话题
        self.subscription = self.create_subscription(
            CompressedImage,
            '/esp32_img/compressed',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用的变量警告

        # 初始化 CvBridge
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # 解码 JPEG 数据为 OpenCV 图像
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_image is not None:
            # 显示图像
            cv2.imshow('Image', cv_image)
            cv2.waitKey(1)  # 等待 1 毫秒，以便窗口更新

def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass

    # 清理资源
    image_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()