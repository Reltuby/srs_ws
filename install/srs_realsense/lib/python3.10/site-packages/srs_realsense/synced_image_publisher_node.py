import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import numpy as np

'''
Image publisher node that ensures depth and color images are synced, This is not technically required
also allows for any preprocessing of images before being sent to the CNN
Currently only publishes a black image to the colorimage topic when the UR5 arm is not ready to pick

the published imgaes are in an image format
'''


class SyncedImagePublisherNode(Node):
    def __init__(self):
        super().__init__('synced_image_publisher')

        #Quality of Service (QoS) for Durability

        qos_profile = QoSProfile(depth = 10)
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        #subscribers
        self.color_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.trigger_sub = self.create_subscription(Bool, '/trigger_publish', self.trigger_callback, qos_profile)

        #synchornise color and depth

        self.sync = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.image_callback)

        self.color_publisher = self.create_publisher(Image, '/processed/color_image', qos_profile)
        self.depth_publisher = self.create_publisher(Image, '/processed/depth_image', qos_profile)

        self.bridge = CvBridge()
        self.should_publish = False

    def trigger_callback(self, msg: Bool):
        #callback for trigger topic
        self.should_publish = msg.data
        self.get_logger().info(f"Publishing Enabled: {self.should_publish}")

    #This node is mostly redundant as it was made before I had a proper understanding of what needed to be done before giving an image to the YOLO CNN (nothing)
    def image_callback(self, color_msg: Image, depth_msg: Image):
        #callback for synced color and depth messages
        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

            black_image = np.zeros((500,500,3), dtype=np.uint8)
            '''
            Can do any preprocessing that is wanted/required in this section and will keep the images synced up
            I am currently unsure if any needs to be done but i will add this section for ease of editing if the need arises
            '''

            #published synced frame only if trigger is set
            if self.should_publish:
                self.color_publisher.publish(self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8'))
                self.depth_publisher.publish(self.bridge.cv2_to_imgmsg(depth_image, encoding='passthrough'))
                #self.get_logger().info("Published Synchronized Images.")
            else:
                #if not triggered then publish the blakc image so no centroids are detected randomly.
                self.color_publisher.publish(self.bridge.cv2_to_imgmsg(black_image, encoding='bgr8'))
                self.depth_publisher.publish(self.bridge.cv2_to_imgmsg(depth_image, encoding='passthrough'))
                

        except Exception as e:
            self.get_logger().error(f"error Processing Images: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SyncedImagePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()