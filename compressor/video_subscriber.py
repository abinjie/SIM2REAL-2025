import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_compressed',  # 订阅压缩图像
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)  # 将压缩数据转换为 NumPy 数组
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # 解码图像数据
        if image is not None:  # 确保图像解码成功
            cv2.imshow("Compressed Video", image)  # 显示图像
            cv2.waitKey(1)  # 等待 1 毫秒，刷新窗口

def main(args=None):
    rclpy.init(args=args)
    node = VideoSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
