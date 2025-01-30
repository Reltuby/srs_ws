from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    #declare launch argument for final picking step
    picking_method_arg = DeclareLaunchArgument(
        'picking_method',
        default_value='individual',
        description='Choose whether to pick all detected kiwifruit, or pick individually'
    )

    picking_method_nodes = {
        'all': 'fruit_pick_all',
        'individual': 'fruit_pick_individual'
    }

    selected_picking_method = Node(
        package='srs_control',
        executable=picking_method_nodes[LaunchConfiguration('picking_method')],
        name='fruit_picker',
        output='screen'
    )


    # Declare a launch argument for selecting the gripper orientation strategy
    gripper_orientation_arg = DeclareLaunchArgument(
        'gripper_orientation',
        default_value='basic',  # Default strategy
        description='Choose gripper orientation strategy: basic, farthest_centroid, or z_order'
    )

    # Define the available gripper orientation nodes
    gripper_orientation_nodes = {
        'basic': 'gripper_orientation_basic',
        'farthest_centroid': 'gripper_orientation_farthest_centroid',
        'z_order': 'gripper_orientation_z_order'
    }

    # Select the gripper orientation executable based on the launch argument
    selected_gripper_orientation_node = Node(
        package='picking_info',
        executable=gripper_orientation_nodes[LaunchConfiguration('gripper_orientation')],
        name='gripper_orientation',
        output='screen'
    )

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
            {'robot_description': open('/home/colby/urdf_final.urdf').read()}
        ]
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
        gripper_orientation_arg,
        picking_method_arg,
        realsense_launch,
        synced_image_publisher_node,
        yolo_prediction_node,
        depth_to_point_node,
        selected_gripper_orientation_node,
        selected_picking_method,
        coord_transform_node,
        robot_state_publisher,
        rviz2
    ])
