import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json
import os

'''node meant to save the detected kiwifruit in the field for later lab testing
Currently does not work in field, I think this is because the robot joint states are not being published when it is used
Solution is to either make a new node to publish joint states which only runs in tandem with this node or to connect to rtde_io in this node

Other changes could be to make the node run once (Make it a service?) and change the naming, otherwise you need to rename each file it saves'''

class SaveTransformedCoords(Node):
    def __init__(self):
        super().__init__('save_transformed_coords')

        # Subscribe to transformed centroid topic
        self.subscription = self.create_subscription(
            String,
            '/transformed_centroid_coord',
            self.listener_callback,
            10
        )

        self.trigger_publisher = self.create_publisher(Bool, '/trigger_publish', 1)
        self.timer = self.create_timer(0.1, self.publish_trigger)


        # Define the directory and file path
        self.directory = "/home/colby/srs_ws/cluster_coords"
        self.file_path = os.path.join(self.directory, "transformed_centroids.txt")

        # Ensure the directory exists
        os.makedirs(self.directory, exist_ok=True)

        self.get_logger().info(f'Saving transformed coordinates to {self.file_path}')


    def publish_trigger(self):
        try:
            msg = Bool()
            msg.data = True
            self.trigger_publisher.publish(msg)
            self.get_logger().info('trigger publish = true')

        except Exception as e:
            self.get_logger().error(f"Failed to trigger Publish: {e}")



    def listener_callback(self, msg):
        try:
            # Parse JSON from message
            transformed_data = json.loads(msg.data)

            # Write to file
            with open(self.file_path, 'w') as file:
                json.dump(transformed_data, file, indent=4)

            self.get_logger().info(f'Transformed data saved to {self.file_path}')
        
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SaveTransformedCoords()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
