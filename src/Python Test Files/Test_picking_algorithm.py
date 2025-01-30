import numpy as np
import json
from sklearn.cluster import DBSCAN
from scipy.spatial.distance import euclidean

#input data as outputted by pixel to point
json_input = '{"0": [], "1": [[0,1,1],[0.5,0.6,1],[0,0.9,1],[0.05,0.95,1],[0.4,0.55,1],[0.35,0.45,1],[0.35,0.5,1],[0.2,0.2,1],[1,0.8,1],[0.7,0.2,1],[0.5,0.3,1]]}'

#parse json string
data = json.loads(json_input)

#extract kiwfruit coordinates (class 1 form json data)
kiwfruit_coordinates = np.array(data["1"])

#perform clustering with 110mm threshold (eps=0.11 meters)
clustering = DBSCAN(eps=0.11, min_samples=1).fit(kiwfruit_coordinates)

#organise the clustered points
clusters = {}
for idx, label in enumerate (clustering.labels_):
    if label not in clusters:
        clusters[label] = []
    clusters[label].append(kiwfruit_coordinates[idx])

#function to calculate the rotation angle of the gripper
def calculate_rotation_angle(current_fruit, neighbors):
    """
    Calculate the rotation angle (around Z-axis) for the gripper based on the 'neighbors' position

    Args:
clustering = DBSCAN(eps=0.11, min_samples=1).fit(kiwfruit_coordinates)
        current_fruit (np.array): the position of the kiwifruit to pick
        neighbors (list of np.array): list of poistions of adjascent kiwifruits.

    Returns:
    float: The angle (in radians) of rotation around the Z-axis
    """

    #calculate the centroid of the neigboring kiwifruit
    if len(neighbors) > 0:
        centroid = np.mean(neighbors, axis=0)

        #find the direction vector to the centroid from the current kiwifruit
        direction_vector = centroid - current_fruit
        
        #Normalize the diretion vector
        direction_vector /= np.linalg.norm(direction_vector)

        #calculate the angle between the direction vector and the positive x-axis (in radians)
        angle = np.arctan2(direction_vector[1], direction_vector[0])

        return angle
    else:
        return 0.0
    
#function to calculate picking order based on proximity
def calculate_picking_order(cluster, start_point):
    order = []
    current_point = start_point
    while cluster:
        #find the nearest kiwifruit
        nearest = min(cluster, key=lambda p: euclidean(p, current_point))
        order.append(nearest)
        cluster = [p for p in cluster if not np.array_equal(p, nearest)]
        current_point = nearest

    return order

#function to generate the picking plan
def calculate_picking_plan(clusters):
    picking_plan = []
    start_point = [0,0,0] #start position of robotic arm (change to get the end effectors actual coordinate)

    for cluster_label, points in clusters.items():
        #get the picking order for the current cluster
        picking_order = calculate_picking_order(points, start_point)

        for point in picking_order:
            #find neighbors in the same cluster (within 110mm)
            neighbors = [p for p in points if not np.array_equal(p, point) and euclidean(point, p) <= 0.11]

            #calculate the grippers safe roation for this kiwifruit
            rotation = calculate_rotation_angle(point, neighbors)

            picking_plan.append({
                'position': point,
                'rotation': rotation
            }) 
    return picking_plan

#generate the picking plan
picking_plan = calculate_picking_plan(clusters)

#output the pikcing plan with positions and gripper rotations
for plan in picking_plan:
    print(f"Pick kiwifruit at {plan['position']} with rotation {plan['rotation']} radians")


