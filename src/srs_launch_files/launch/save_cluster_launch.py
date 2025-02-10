from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription

'''
launch file that was intended to save the coordinates of detected kiwifruit in the field for later lab testing
more testing required as it did not work for field test
However it is likely that the issue is with the save cluster node not the launch file.
'''

def generate_launch_description():
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'examples/align_depth',
                'rs_align_depth_launch.py'
            )
        )
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': open('/home/colby/srs_ws/urdf_final.urdf').read()}  
        ]
    )

    save_cluster_coords = Node(
        package='picking_info',
        executable='save_cluster_coords',
        name='save_cluster_coords',
        output='screen'
    )
    
    synced_image_publisher_node = Node(
        package='srs_realsense',
        executable='synced_image_publisher_node',
        name='synced_image_publisher',
        output='screen'
    )
    
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    yolo_prediction_node = Node(
        package='yolo_prediction',
        executable='yolo_prediction_node',
        name='yolo_prediction',
        output='screen'
    )
    
    depth_to_point_node = Node(
        package='srs_realsense',
        executable='pixel_to_point',
        name='pixel_depth_to_point',
        output='screen'
    )
    
    coord_transform_node = Node(
        package='srs_control',
        executable='coord_transform',
        name='coord_transform',
        output='screen'
    )
    
    return LaunchDescription([
        realsense_launch,
        synced_image_publisher_node,
        yolo_prediction_node,
        depth_to_point_node,
        coord_transform_node,
        robot_state_publisher,
        rviz2,
        save_cluster_coords
    ])
