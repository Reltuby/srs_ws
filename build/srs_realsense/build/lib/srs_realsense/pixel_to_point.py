import rclpy, json
import pyrealsense2 as rs
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge

'''
Node that converts all points detected by the CNN into 3D coordinates using deproject pixel to point function

/centroid_coord is json encoded string in this format where 0 is for calyxes and 1 is for kiwifruit
{
  "0": [[1.23, 2.34, 3.45], [4.56, 5.67, 6.78]],
  "1": [[7.89, 8.90, 9.01]]
}
'''

class DepthToPointNode(Node):
    def __init__(self):
        super().__init__('depth_to_point_node')

        #initialise CvBridge
        self.bridge = CvBridge()
        self.intrinsics = None
        self.centroids_by_class = None

        #create Subscription do Aligned Depth to Color Topic
        self.create_subscription(
            Image,  #message type
            '/processed/depth_image', #topic to subscribe to
            self.depth_callback,  #callback function
            10 # QoS (Quality of Service)
            )

        self.create_subscription(
            CameraInfo,  #message type
            '/camera/camera/aligned_depth_to_color/camera_info', #topic to subscribe to
            self.camera_info_callback,  #callback function
            10 # QoS (Quality of Service)
            )
        
        self.create_subscription(
            String,
            'centroid_data',
            self.centroid_data_callback,
            10
        )

        self.publisher = self.create_publisher(String, "/centroid_coord", 10)

    #retrieves the camera intrinsics
    def camera_info_callback(self, msg):
        # extract intrinsics from msg
        self.intrinsics = msg.k
        #self.get_logger().info("Camera Intrinsics Recieved")
        #look up intrinsics matrix msg.K

    #load the predicted kiwifruit positions
    def centroid_data_callback(self, msg):
        try:
            self.centroids_by_class = json.loads(msg.data)
            self.get_logger().info(f"Recieved centroids: {self.centroids_by_class}") 
        except json.JSONDecodeError as e:
            self.get_logger().info(f"Failed to parse centroid data: {e}")


    def depth_callback(self, msg):
        #check that camera info is available
        if self.intrinsics is None:
            self.get_logger().warn('Waiting For Camera Intrinsics...')
            return
        #check that centroids have been recieved
        if self.centroids_by_class is None:
            self.get_logger().warn('Waiting For Centroid Data...')
            return
        
        #convert ros Image to numpy array
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        #store all of the ros intrinsics in variables
        intrinsics = rs.intrinsics()
        intrinsics.fx = self.intrinsics[0] #fx    
        intrinsics.fy = self.intrinsics[4] #fy    
        intrinsics.ppx = self.intrinsics[2] #cx    
        intrinsics.ppy = self.intrinsics[5] #cy    
        intrinsics.model = rs.distortion.none
        intrinsics.width = msg.width
        intrinsics.height = msg.height

        all_points_3d = {}

        #Iterate through each class and its centroids
        #converts each pixel into a 3d coordinate
        for cls, centroids in self.centroids_by_class.items():
            class_points = []
            for (pixel_x, pixel_y) in centroids:
                #ensure pixel is valid
                if 0 <= pixel_x < depth_image.shape[1] and 0 <= pixel_y < depth_image.shape[0]:
                    depth_value = depth_image[int(pixel_y), int(pixel_x)] * 0.001
                    if depth_value == 0:
                        self.get_logger().warn(f"No depth value at {pixel_x}, {pixel_y} for class {cls}")
                        continue
                    #if valid pixel convert to point
                    point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [pixel_x, pixel_y], depth_value)
                    
                    #add to list
                    class_points.append((point_3d[0], point_3d[1], point_3d[2]))
                    self.get_logger().info(f"Class {cls}: 3D coordinates at ({pixel_x}, {pixel_y}) -> x = {point_3d[0]:.2f}, y={point_3d[1]:.2f}, z={point_3d[2]:.2f} meters")
                else:
                    self.get_logger().warn(f"Centroid ({pixel_x}, {pixel_y}) out of bounds for depth image")

            #store the points for this class        
            all_points_3d[cls] = class_points

        #publish the 3d coordinates
        try:
            message_data = json.dumps(all_points_3d)
            msg = String()
            msg.data = message_data
            self.publisher.publish(msg)
            self.get_logger().info(f"Published 3D coordinates: {message_data}")           
        except Exception as e:
            self.get_logger().error(f"Failed to publihs 3D coordinates: {e}")


def main(args = None):
    rclpy.init(args=args)
    node = DepthToPointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting Down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()