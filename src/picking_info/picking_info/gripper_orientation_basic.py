import rclpy, json
import numpy as np
from rclpy.node import Node
from sklearn.cluster import DBSCAN
from scipy.spatial import distance
from std_msgs.msg import String

class GripperOrientationNode(Node):
    def __init__(self):
        super().__init__('gripper_orientation')
        self.subscription = self.create_subscription(
            String,
            '/transformed_centroid_coord',
            self.centroid_data_callback,
            10
        )
        self.publisher = self.create_publisher(String, '/gripper_orientation', 10)

    def centroid_data_callback(self, msg):
        try:
            data = json.loads(msg.data)
            kiwifruit_coordinates = np.array(data["1"])

            # If no centroids, publish empty picking plan
            if kiwifruit_coordinates.size == 0:
                self.get_logger().warn("No centroids found, publishing empty picking plan.")
                self.publish_picking_plan([])  # Publish an empty list
                return  # Stop further processing
            
            picking_plan = self.plan_kiwifruit_picking(kiwifruit_coordinates)
            self.publish_picking_plan(picking_plan)

        except Exception as e:
            self.get_logger().error(f"error processing centroid data: {e}")

    def publish_picking_plan(self, picking_plan):
        picking_plan_str = json.dumps(picking_plan)
        self.publisher.publish(String(data=picking_plan_str))
        self.get_logger().info(f"Published picking plan: {picking_plan_str}")

    def group_kiwifruit(self, kiwifruit_positions, eps=0.11, min_samples=1):
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
        # sort by z then y (lowest first)
        centroids = sorted(centroids, key=lambda c: (c[2], c[1]))
        remaining_centroids = centroids.copy()
        picking_plan = []
        gripper_rotation = -1.57

        while remaining_centroids:
            current = remaining_centroids[0]
            picking_plan.append({'position': current.tolist(), 'rotation': gripper_rotation})
            remaining_centroids.pop(0)

        return picking_plan
    
    def plan_kiwifruit_picking(self, kiwifruit_positions, eps=0.11, min_samples=1):
        clusters = self.group_kiwifruit(kiwifruit_positions, eps, min_samples)
        picking_plan = []

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
