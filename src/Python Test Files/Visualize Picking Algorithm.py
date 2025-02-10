import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial import distance
import matplotlib.pyplot as plt

'''test file to evaluate new picking algorithms 
edit as necasary if making a new algorithm'''


# Group kiwifruit into clusters
def group_kiwifruit(kiwifruit_positions, eps=0.11, min_samples=1):
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

# Find the largest empty space angle around each fruit
def find_largest_empty_angle(centroids, radius=0.11):
    empty_space_scores = []  # To store empty space scores for each fruit
    for i, current in enumerate(centroids):
        # Compute distances to all other fruit within the radius
        distances = distance.cdist([current], centroids)[0]
        neighbors = [centroids[j] for j, d in enumerate(distances) if 0 < d <= radius]
        if not neighbors:
            empty_space_scores.append((current, 360))  # No neighbours → max empty space
            continue
        # Compute angles of all neighbours relative to the current fruit
        angles = []
        for neighbor in neighbors:
            vector = np.array(neighbor[:2]) - np.array(current[:2])
            angle = np.arctan2(vector[1], vector[0])  # Angle in radians
            angles.append(angle)
        # Sort angles and compute the largest gap
        angles = sorted(angles)
        angle_gaps = np.diff(angles)  # Difference between consecutive angles
        largest_gap = max(angle_gaps, default=0)
        # Wraparound case: Check gap between last and first (circular space)
        full_circle_gap = (2 * np.pi + angles[0]) - angles[-1]
        largest_gap = max(largest_gap, full_circle_gap)
        empty_space_scores.append((current, largest_gap))

    # Sort by largest empty angle first
    empty_space_scores.sort(key=lambda x: x[1], reverse=True)
    sorted_centroids = [item[0] for item in empty_space_scores]
    return sorted_centroids, empty_space_scores

def calculate_picking_order(centroids, radius=0.11, alpha=1):
    # Get the largest empty space angles and sort by that
    sorted_centroids, empty_space_scores = find_largest_empty_angle(centroids, radius)
    picking_plan = []
    gripper_rotation = -1.57  # Default orientation

    # Get the minimum and maximum z-values for normalisation
    z_min = min([centroid[2] for centroid in sorted_centroids])
    z_max = max([centroid[2] for centroid in sorted_centroids])

    # Calculate combined score based on empty space and z-height
    combined_scores = []
    for current in sorted_centroids:
        # Normalise the z value between 0 and 1
        normalised_z = (current[2] - z_min) / (z_max - z_min) if z_max != z_min else 0
        
        # Debug: check the type and value of empty_space_angle
        empty_space_tuple = next(item[1] for item in zip(sorted_centroids, empty_space_scores) if np.allclose(item[0], current))
        print(f"Empty space for {current}: {empty_space_tuple}")
        
        # If empty_space_tuple is a tuple, we assume the second element is the angle.
        empty_space_angle = empty_space_tuple[1] if isinstance(empty_space_tuple, tuple) else float(empty_space_tuple)
        
        # Combine the scores (higher score means lower priority)
        score = alpha * normalised_z + (1 - alpha) * empty_space_angle
        combined_scores.append((current, score))

    # Sort by combined score (lower score comes first)
    combined_scores.sort(key=lambda x: x[1])  # Sort by score (lowest score first)
    sorted_centroids = [item[0] for item in combined_scores]

    while sorted_centroids:
        current = sorted_centroids[0]

        # Find neighbours within the defined radius for gripper orientation
        distances = distance.cdist([current], sorted_centroids)[0]
        neighbors = [i for i, d in enumerate(distances) if d <= radius and i != 0]

        if neighbors:
            neighbor_coords = [sorted_centroids[i] for i in neighbors]
            avg_neighbor = np.mean(neighbor_coords, axis=0)
            orientation_vector = avg_neighbor[:2] - np.array(current[:2])
            perpendicular_vector = np.array([-orientation_vector[1], orientation_vector[0]])
            gripper_rotation = np.arctan2(perpendicular_vector[1], perpendicular_vector[0])

        picking_plan.append({'position': current, 'rotation': gripper_rotation})
        sorted_centroids.pop(0)  # Remove picked fruit

    return picking_plan



# Plan kiwifruit picking by grouping and considering sorting by both z-height and empty space
def plan_kiwifruit_picking(kiwifruit_positions, eps=0.11, min_samples=1, neighbor_radius=0.11, alpha=0):
    clusters = group_kiwifruit(kiwifruit_positions, eps, min_samples)
    picking_plan = []
    for cluster in clusters:
        cluster_plan = calculate_picking_order(cluster, neighbor_radius, alpha)
        picking_plan.extend(cluster_plan)
    return picking_plan

def visualize_picking_plan_3d(picking_plan):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Extract positions and rotations from the picking plan
    positions = [plan['position'] for plan in picking_plan]
    rotations = [plan['rotation'] for plan in picking_plan]

    # Extract X, Y, and Z coordinates
    xs, ys, zs = zip(*positions)

    # Plot kiwifruits in 3D
    ax.scatter(xs, ys, zs, c='g', label='Kiwifruits', s=100)

    # Annotate each fruit with its picking order
    for i, (x, y, z, rot) in enumerate(zip(xs, ys, zs, rotations)):
        ax.text(x, y, z, str(i + 1), color='yellow', fontsize=10, ha='center', va='center')

        # Gripper orientation (represented by a simple line)
        dx = 0.05 * np.cos(rot)
        dy = 0.05 * np.sin(rot)
        ax.plot([x - dx / 6, x + dx / 6], [y - dy / 6, y + dy / 6], [z, z], c='r')

    # Set labels and title
    ax.set_xlabel('X-axis (m)')
    ax.set_ylabel('Y-axis (m)')
    ax.set_zlabel('Z-axis (m)')
    ax.set_title('3D Picking Order with Gripper Orientation')

    # Show legend
    ax.legend()

    # Display the 3D plot
    plt.show()

kiwifruit_cluster = [
    [0.10, 0.15, 0.121],  # Central fruit
    [0.12, 0.17, 0.122],
    [0.08, 0.16, 0.113],
    [0.11, 0.13, 0.124],
    [0.09, 0.18, 0.105],
    [0.14, 0.16, 0.126],
    [0.07, 0.14, 0.117],
    [0.12, 0.14, 0.128],
    [0.13, 0.18, 0.129],
    [0.11, 0.19, 0.120],
    [0.10, 0.17, 0.111],
    [0.08, 0.13, 0.102],
    [0.09, 0.16, 0.123],
    [0.07, 0.17, 0.114],
    [0.13, 0.15, 0.125]
]

# Generate the picking plan
picking_plan = plan_kiwifruit_picking(kiwifruit_cluster)

# Output the picking plan with positions and gripper rotations
print("Picking Plan:")
for step in picking_plan:
    print(f"Position: {step['position']} meters, Rotation {step['rotation']} radians")

# Visualise the picking order in 3D
visualize_picking_plan_3d(picking_plan)
