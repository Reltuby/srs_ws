import rclpy, json
import numpy as np
from rclpy.node import Node
from sklearn.cluster import DBSCAN
from scipy.spatial import distance
from std_msgs.msg import String

'''
This node sorts the detected kiwifruit by their z order picking the lowest kiwifruit first
The gripper orientation is calculated by finding a centroid of the target fruits 'neighbors' and picking perpindicular to that vector

Calyx detection is supported up to this step, but only the detected kiwifruit are evaluated from here
'''

class GripperOrientationNode(Node):
    def __init__(self):
        super().__init__('gripper_orientation')

        #create subscriber
        self.subscription = self.create_subscription(
            String,
            '/transformed_centroid_coord',
            self.centroid_data_callback,
            10
        )
        #create publisher
        self.publisher = self.create_publisher(String, '/gripper_orientation', 10)

    def centroid_data_callback(self, msg):
        '''load centroid data'''
        try:
            #decode json
            data = json.loads(msg.data)
            #only get class 1 (kiwifruit) ignore calyxes (class 0)
            kiwifruit_coordinates = np.array(data["1"])

            # If no centroids, publish empty picking plan
            if kiwifruit_coordinates.size == 0:
                self.get_logger().warn("No centroids found, publishing empty picking plan.")
                self.publish_picking_plan([])  # Publish an empty list
                return  # Stop further processing
            
            #calculate pikcing order
            picking_plan = self.plan_kiwifruit_picking(kiwifruit_coordinates)
            self.publish_picking_plan(picking_plan)

        except Exception as e:
            self.get_logger().error(f"error processing centroid data: {e}")

    def publish_picking_plan(self, picking_plan):
        '''publish picking order'''
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
        
    
    def calculate_picking_order(self, centroids, radius=0.11):
        '''sort kiwifruit from lowest to highest'''
        centroids = sorted(centroids, key=lambda c: (c[2], c[1]))

        #copy centroid list
        remaining_centroids = centroids.copy()
        picking_plan = []

        #initial orientation
        gripper_rotation = -1.57    

        #for every kiwifruit calculate the gripper orientation and append to picking order list
        while remaining_centroids:
            current = remaining_centroids[0]
            
            # Find neighbors within the defined radius
            distances = distance.cdist([current], remaining_centroids)[0]
            neighbors = [i for i, d in enumerate(distances) if d <= radius and i != 0]

            #if there are neighbors find their centroid as a vector and pick with the z orientation perpindicular to this
            #otherwise for singular fruit don't adjust orientation
            if neighbors:
                neighbor_coords = [remaining_centroids[i] for i in neighbors]
                avg_neighbor = np.mean(neighbor_coords, axis=0)
                orientation_vector = avg_neighbor[:2] - np.array(current[:2])
                perpindicular_vector = np.array([-orientation_vector[1], orientation_vector[0]])
                gripper_rotation = np.arctan2(perpindicular_vector[1], perpindicular_vector[0])

            #append to list
            picking_plan.append({'position': current.tolist(), 'rotation': gripper_rotation})

            #remove picked fruit so that the gripper orientation calculations remain accurate
            remaining_centroids.pop(0)

        return picking_plan
    
    def plan_kiwifruit_picking(self, kiwifruit_positions, eps=0.11, min_samples=1, neighbor_radius=0.11):
        '''plan all kiwifruit picking, each cluster is sorted individually
        but the order of clusters is not sorted so the model can pick clusters on opposite sides of its view one after the other'''
        #create list of clusters
        clusters = self.group_kiwifruit(kiwifruit_positions, eps, min_samples)
        picking_plan = []

        #for each cluster calculate picking order and append to final order
        for cluster in clusters:
            cluster_plan = self.calculate_picking_order(cluster, neighbor_radius)
            picking_plan.extend(cluster_plan)

        return picking_plan
    
def main(args=None):
    rclpy.init(args=args)
    node = GripperOrientationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
