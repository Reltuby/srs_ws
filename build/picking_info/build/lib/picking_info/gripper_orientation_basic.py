import rclpy, json
import numpy as np
from rclpy.node import Node
from sklearn.cluster import DBSCAN
from std_msgs.msg import String

'''
This node sorts the detected kiwifruit by their z order picking the lowest kiwifruit first
It does no calculation for the gripper orientation and will always pick with a gripper rotation of -90 degrees

Calyx detection is supported up to this step, but only the detected kiwifruit are evaluated from here
'''

class GripperOrientationNode(Node):
    def __init__(self):
        super().__init__('gripper_orientation')
        #subscribe to centroid topic
        self.subscription = self.create_subscription(
            String,
            '/transformed_centroid_coord',
            self.centroid_data_callback,
            10
        )
        #publish to new topic
        self.publisher = self.create_publisher(String, '/gripper_orientation', 10)


    def centroid_data_callback(self, msg):
        '''loads the centroid data'''
        try:
            #decode json message
            data = json.loads(msg.data)
            #only get the kiwifruit from the data this is where the calyx is ignored
            kiwifruit_coordinates = np.array(data["1"])

            # If no centroids, publish empty picking plan
            if kiwifruit_coordinates.size == 0:
                self.get_logger().warn("No centroids found, publishing empty picking plan.")
                self.publish_picking_plan([])  # Publish an empty list
                return  # Stop further processing
            
            #plan picking order
            picking_plan = self.plan_kiwifruit_picking(kiwifruit_coordinates)
            self.publish_picking_plan(picking_plan)

        except Exception as e:
            self.get_logger().error(f"error processing centroid data: {e}")

    def publish_picking_plan(self, picking_plan):
        '''publisher for the picking plan'''
        picking_plan_str = json.dumps(picking_plan)
        self.publisher.publish(String(data=picking_plan_str))
        self.get_logger().info(f"Published picking plan: {picking_plan_str}")

    def group_kiwifruit(self, kiwifruit_positions, eps=0.11, min_samples=1):
        '''group the kiwifruit into clusters if within a distance of 0.11m. Minimum cluster size being 1'''
        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(kiwifruit_positions)
        labels = clustering.labels_
        unique_labels = set(labels)
        clusters = []
        for label in unique_labels:
            if label == -1:
                continue
            cluster = [kiwifruit_positions[i] for i in range(len(kiwifruit_positions)) if labels[i] == label]
            clusters.append(cluster)
        return clusters
        
    
    def calculate_picking_order(self, centroids):
        '''define how the kiwifruit are sorted in this case by their z position, then y position if they are the same z'''
        #sort centroids
        centroids = sorted(centroids, key=lambda c: (c[2], c[1]))
        #copy the list of centroids
        remaining_centroids = centroids.copy()
        picking_plan = []

        #set default gripper orientation to -90 degrees
        gripper_rotation = -1.57

        #for every kiwifruit positions combine the position and rotation
        while remaining_centroids:
            current = remaining_centroids[0]
            picking_plan.append({'position': current.tolist(), 'rotation': gripper_rotation})
            #remove calculated kiwifruit from the list
            remaining_centroids.pop(0)

        return picking_plan
    
    def plan_kiwifruit_picking(self, kiwifruit_positions, eps=0.11, min_samples=1):
        '''plan all kiwifruit picking, each cluster is sorted individually
        but the order of clusters is not sorted so the model can pick clusters on opposite sides of its view one after the other'''
        #create list of clusters
        clusters = self.group_kiwifruit(kiwifruit_positions, eps, min_samples)
        picking_plan = []

        #for each cluster calculate the picking order and append to master picking order
        for cluster in clusters:
            cluster_plan = self.calculate_picking_order(cluster)
            picking_plan.extend(cluster_plan)

        return picking_plan
    
def main(args=None):
    rclpy.init(args=args)
    node = GripperOrientationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
