import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class ImageCompressor(Node):
    def __init__(self):
        super().__init__('image_compressor')
        
        # 订阅原始图像
        self.subscription = self.create_subscription(
            Image, 
            '/mmk2/head_camera/color/image_raw',  
            self.image_callback, 
            10
        )
        
        # 发布压缩图像
        self.publisher = self.create_publisher(
            CompressedImage, 
            '/camera/image_compressed', 
            10
        )
        
        self.bridge = CvBridge()
        self.get_logger().info('Image Compressor Node Started')

    def image_callback(self, msg):
        try:
            # 转换 ROS 2 图像消息为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 进行 JPEG 压缩
            encode_param = [cv2.IMWRITE_JPEG_QUALITY, 50]  # 50% 质量
            _, encoded_image = cv2.imencode('.jpg', cv_image, encode_param)
            
            # 转换回 ROS 2 `CompressedImage` 消息
            compressed_msg = CompressedImage()
            compressed_msg.format = "jpeg"
            compressed_msg.data = encoded_image.tobytes()
            
            # 发布压缩后的图像
            self.publisher.publish(compressed_msg)
        
        except Exception as e:
            self.get_logger().error(f"Error compressing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageCompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Image Compressor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
