from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction

'''
Launch file for entire picking system
launches all required nodes for automated kiwifruit picking
includes chagneable parameters for the gripper orientation/picking order (z_order basic farthest_centroid), node and picking method (pick_all pick_individual)
'''


#function for choosing which picking method you want
def launch_picking_method_node(context, *args, **kwargs):
    picking_method_value = context.launch_configurations['picking_method']
    picking_method_nodes = {
        'all': 'fruit_pick_all',
        'individual': 'fruit_pick_individual'
    }
    return [Node(
        package='srs_control',
        executable=picking_method_nodes.get(picking_method_value, 'fruit_pick_individual'),
        name='fruit_picker',
        output='screen',
        parameters=[{'picking_method': picking_method_value}]
    )]

#function for choosing how the sorting is done 
def launch_gripper_orientation_node(context, *args, **kwargs):
    gripper_orientation_value = context.launch_configurations['gripper_orientation']
    gripper_orientation_nodes = {
        'basic': 'gripper_orientation_basic',
        'farthest_centroid': 'gripper_orientation_farthest_centroid',
        'z_order': 'gripper_orientation_z_order'
    }
    return [Node(
        package='picking_info',
        executable=gripper_orientation_nodes.get(gripper_orientation_value, 'gripper_orientation_basic'),
        name='gripper_orientation',
        output='screen',
        parameters=[{'gripper_orientation': gripper_orientation_value}]
    )]


def generate_launch_description():
    #launches the node for the desired picking method
    picking_method_arg = DeclareLaunchArgument(
        'picking_method',
        default_value='individual',
        description='Choose whether to pick all detected kiwifruit, or pick individually'
    )
    
    #launches the desired fruit sorting node
    gripper_orientation_arg = DeclareLaunchArgument(
        'gripper_orientation',
        default_value='basic',
        description='Choose gripper orientation strategy: basic, farthest_centroid, or z_order'
    )
    
    #launches the align_depth to color example launch file from the realsense developers
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'examples/align_depth',
                'rs_align_depth_launch.py'
            )
        )
    )
    



    '''create a description for each node that you want to launch'''
    #if you want to edit/add nodes just follow formating of below examples

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': open('/home/colby/srs_ws/urdf_final.urdf').read()}  
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
    
    '''must call additional nodes when they are added or they will not be launched'''
    return LaunchDescription([
        gripper_orientation_arg,
        picking_method_arg,
        realsense_launch,
        synced_image_publisher_node,
        yolo_prediction_node,
        depth_to_point_node,
        OpaqueFunction(function=launch_gripper_orientation_node),
        OpaqueFunction(function=launch_picking_method_node),
        coord_transform_node,
        robot_state_publisher,
        rviz2
    ])
