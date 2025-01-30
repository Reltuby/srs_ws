import rclpy, json
from rclpy.node import Node
from tf2_geometry_msgs.tf2_geometry_msgs import PointStamped
from tf2_ros import TransformListener, Buffer
from std_msgs.msg import String

class CoordTransform(Node):
    def __init__(self):
        super().__init__('coord_transform')

        #create TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #subscribe to centroid coord topic
        self.centroid_sub = self.create_subscription(String, '/centroid_coord', self.centroid_callback, 10)

        #publisher for the transformed coordinates
        self.transformed_pub = self.create_publisher(String, '/transformed_centroid_coord', 10)

        self.get_logger().info('Coord Transform Node Initialized')

    def centroid_callback(self, msg):
        try:
            #decode JSON
            centroids = json.loads(msg.data)

            #lookup the transform from tcp to base
            transform = self.tf_buffer.lookup_transform('base', 'camera_depth_optical_frame', rclpy.time.Time())
            
            transformed_centroids = {}

            for key, coords_list in centroids.items():
                transformed_coords = []
                for coord in coords_list:
                    #create a PointStamped object for each coordinate
                    point = PointStamped()
                    
                    point.header.frame_id = 'camera_depth_optical_frame'
                    point.point.x, point.point.y, point.point.z = coord

                    #transform point
                    transformed_point = self.tf_buffer.transform(point, 'base')
                    transformed_coords.append([
                        transformed_point.point.x,
                        transformed_point.point.y,
                        transformed_point.point.z
                    ])

                transformed_centroids[key] = transformed_coords

            #encode transformed coords into json and publish
            transformed_msg = String()
            transformed_msg.data = json.dumps(transformed_centroids)
            self.transformed_pub.publish(transformed_msg)

            self.get_logger().info(f'transformed Centroids: {transformed_msg.data}')

        except Exception as e:
            self.get_logger().error(f'Error in centroid transformation: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CoordTransform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()