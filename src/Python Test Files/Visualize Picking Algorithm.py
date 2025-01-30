import numpy as np
import json
from sklearn.cluster import DBSCAN
from scipy.spatial import distance
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def group_kiwifruit(kiwifruit_positions, eps=0.11, min_samples=1):
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(kiwifruit_positions)
    labels = clustering.labels_ #cluster IDS for each fruit
    unique_labels = set(labels)

    clusters = []

    for label in unique_labels:
        if label == -1:
            #noise point (not part of a cluster)
            continue
        cluster = [kiwifruit_positions[i] for i in range(len(kiwifruit_positions)) if labels[i] == label]
        clusters.append(cluster)

    return clusters

def calculate_picking_order(centroids, radius=0.11):
    # sort by z then y (lowest first)
    centroids = sorted(centroids, key=lambda c: (c[2], c[1]))
    remaining_centroids = centroids.copy()
    picking_plan = []

    while remaining_centroids:
        #always pick first available kiwifruit
        current = remaining_centroids[0]

        #find adjascent kiwifruit within the radius
        distances = distance.cdist([current], remaining_centroids)[0]
        neighbors = [i for i, d in enumerate(distances) if d <= radius and i != 0]

        if neighbors:
            neighbor_coords = [remaining_centroids[i] for i in neighbors]
            avg_neighbor = np.mean(neighbor_coords, axis=0)
            orientation_vector = avg_neighbor[:2] - np.array(current[:2])   #points to centroid of neighboring kiwifruit
            perpindicular_vector = np.array([-orientation_vector[1], orientation_vector[0]])    #takes perpindicular vector
            gripper_rotation = np.arctan2(perpindicular_vector[1], perpindicular_vector[0])

        picking_plan.append({'position': current, 'rotation': gripper_rotation})
        remaining_centroids.pop(0)
    
    return picking_plan

def plan_kiwifruit_picking(kiwifruit_positions, eps=0.11, min_samples=1, neighbor_radius=0.11):
    clusters = group_kiwifruit(kiwifruit_positions, eps, min_samples)
    picking_plan = []

    for cluster in clusters:
        cluster_plan = calculate_picking_order(cluster, neighbor_radius)
        picking_plan.extend(cluster_plan)

    return picking_plan


def visualize_picking_plan_2d_analysis(picking_plan, target_index=1, neighbor_radius=0.11):
    fig, axs = plt.subplots(2, 1, figsize=(8, 12), facecolor='none')

    # Extract positions and rotations
    positions = [plan['position'] for plan in picking_plan]
    rotations = [plan['rotation'] for plan in picking_plan]

    # Top-down view (XY plane) - First subplot
    ax_xy_topdown = axs[0]
    xs, ys, _ = zip(*positions)
    ax_xy_topdown.scatter(xs, ys, c='g', label='Kiwifruits', s=250)

    # Annotate picking order and add gripper orientation
    for i, (x, y, rot) in enumerate(zip(xs, ys, rotations)):
        ax_xy_topdown.text(x, y, str(i + 1), color='yellow', fontsize=14, ha='center', va='center')
        # Add gripper orientation as a line segment
        dx = 0.1 * np.cos(rot)
        dy = 0.1 * np.sin(rot)
        ax_xy_topdown.plot([x - dx / 6, x + dx / 6], [y - dy / 6, y + dy / 6], c='r', label='Gripper Orientation' if i == 0 else '')

    ax_xy_topdown.set_xlabel('X-axis (m)')
    ax_xy_topdown.set_ylabel('Y-axis (m)')
    ax_xy_topdown.set_title('Picking Order with Gripper Orientation')
    ax_xy_topdown.legend()
    ax_xy_topdown.grid()
    ax_xy_topdown.patch.set_alpha(0)  # Set subplot background transparent

    # Analysis of Gripper Orientation (XY plane) - Second subplot
    ax_xy_analysis = axs[1]
    target_position = positions[target_index]
    target_rotation = rotations[target_index]

    # Plot all unpicked fruits
    ax_xy_analysis.scatter(xs[target_index + 1:], ys[target_index + 1:], c='g', label='Unpicked Fruits', s=250)

    # Highlight the target fruit
    ax_xy_analysis.scatter([target_position[0]], [target_position[1]], c='r', label='Target Fruit', s=250)

    # Calculate distances from the target fruit to remaining unpicked fruits
    remaining_positions = positions[target_index + 1:]
    remaining_coords = np.array(remaining_positions)
    distances = np.linalg.norm(remaining_coords[:, :2] - np.array(target_position[:2]), axis=1)
    neighbors = [
        (i, dist) for i, dist in enumerate(distances) if dist <= neighbor_radius
    ]

    # Plot neighboring fruits and calculate the centroid of neighbors
    neighbor_coords = []
    for i, dist in neighbors:
        neighbor_pos = remaining_positions[i]
        neighbor_coords.append(neighbor_pos[:2])
        ax_xy_analysis.scatter([neighbor_pos[0]], [neighbor_pos[1]], c='b', label='Neighboring Fruit' if i == neighbors[0][0] else '', s=250)
        ax_xy_analysis.text(neighbor_pos[0], neighbor_pos[1]+0.01, f"{dist:.2f}m", fontsize=14, ha='center')

        # Draw line connecting target fruit to neighbor
        ax_xy_analysis.plot(
            [target_position[0], neighbor_pos[0]],
            [target_position[1], neighbor_pos[1]],
            c='gray',
            linestyle='dashed',
        )

    # Calculate centroid of neighbors and gripper orientation vector
    if neighbor_coords:
        centroid = np.mean(neighbor_coords, axis=0)
        orientation_vector = centroid - np.array(target_position[:2])
        perpendicular_vector = np.array([-orientation_vector[1], orientation_vector[0]])

        # Plot centroid of neighbors
        ax_xy_analysis.scatter([centroid[0]], [centroid[1]], c='purple', label='Centroid of Neighbors', s=150)
        ax_xy_analysis.plot(
            [target_position[0], centroid[0]],
            [target_position[1], centroid[1]],
            c='purple',
            linestyle='solid',
            label='To Centroid',
        )

        # Plot gripper orientation perpendicular vector
        ax_xy_analysis.plot(
            target_position[0], target_position[1],
            color='red', label='Gripper Orientation'
        )
    # Adjust boundaries of the analysis plot
    target_x, target_y = target_position[:2]
    boundary_range = 0.15  # Increase this value for a wider range
    ax_xy_analysis.set_xlim(target_x - boundary_range, target_x + boundary_range)
    ax_xy_analysis.set_ylim(target_y - boundary_range, target_y + boundary_range)

    ax_xy_analysis.set_xlabel('X-axis (m)')
    ax_xy_analysis.set_ylabel('Y-axis (m)')
    ax_xy_analysis.set_title('Gripper Orientation Analysis of Fruit 2')
    ax_xy_analysis.legend()
    ax_xy_analysis.grid()
    ax_xy_analysis.patch.set_alpha(0)  # Set subplot background transparent

    # Show the plot
    plt.tight_layout()
    plt.show()





#input data as outputted by pixel to point
json_input = '{"0": [], "1": [[0.15,0.123,0.12],[0,0.143,0.143],[0.05,0.092,0.152],[0.105,0.133,0.131],[0.2,0.2,0.127],[0.023,0.14,0.161],[0.134,0.165,0.138],[0.18,0.143,0.149]]}'
#parse json string
data = json.loads(json_input)
#extract kiwfruit coordinates (class 1 form json data)
kiwfruit_coordinates = np.array(data["1"])

#generate the picking plan
picking_plan = plan_kiwifruit_picking(kiwfruit_coordinates)

#output the pikcing plan with positions and gripper rotations
print("Picking Plan:")
for step in picking_plan:
    print(f"Position: {step['position']} meters, Rotation {step['rotation']} radians")

visualize_picking_plan_2d_analysis(picking_plan)


