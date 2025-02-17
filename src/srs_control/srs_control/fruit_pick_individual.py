import rtde_control, rtde_receive, rtde_io, json, rclpy
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState

'''
UR5 control node that picks one fruit then reevaluates for the next fruit
All uses of ur_rtde or other connection methods to the robot must be done in one node 
as the RTDE ports only allow one connection at a time
'''


class FruitPickerNode(Node):
    def __init__(self):
        super().__init__('fruit_pick_check')

        #declare the robots ip must match what is on the robot
        self.declare_parameter('robot_ip', '192.168.131.8')
        self.robot_ip = self.get_parameter('robot_ip').value

        #change the movement speed and acceleration of the robot
        self.spd = 1
        self.accel = 1

        self.home = False

        # Initialise RTDE Control and Receive and IO interfaces
        self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)
        self.rtde_io = rtde_io.RTDEIOInterface(self.robot_ip)

        #Start all Publishers
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.trigger_publisher = self.create_publisher(Bool, '/trigger_publish', 1)
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Subscribe to gripper_orientation topic
        self.subscription = self.create_subscription(
            String, '/gripper_orientation', self.process_fruit_positions, 10
        )
        self.get_logger().info("Individual Fruit Picker Initialised")

    def publish_joint_states(self):
        """Publishes the current joint states of the robot. This is required so that the transforms can be computed with an accurate robot description"""
        try:
            #get joint positions
            joint_positions = self.rtde_r.getActualQ()
            #convert joint positions to ros message and publish to /joint_states
            if joint_positions:
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = [
                    "shoulder_pan_joint", "shoulder_lift_joint",
                    "elbow_joint", "wrist_1_joint",
                    "wrist_2_joint", "wrist_3_joint"
                ]
                joint_state_msg.position = joint_positions
                self.joint_state_publisher.publish(joint_state_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish joint states: {e}")

    def move_to_pose_and_log(self, pose, action_description):
        """Helper function to move to a pose and log the action."""
        #check that the desired pose is within safety limits
        if self.rtde_c.isPoseWithinSafetyLimits(pose) and self.rtde_c.isJointsWithinSafetyLimits(self.rtde_c.getInverseKinematics(pose)):
            #move TCP linearly with speed and acceleration set on initialization
            self.rtde_c.moveL(pose, speed=self.spd, acceleration=self.accel)
            #log what was done
            self.get_logger().info(f"{action_description} at x={pose[0]}, y={pose[1]}, z={pose[2]} with orientation {pose[5]}")
            return True
        else:
            self.get_logger().warn(f"Pose Unreachable: x={pose[0]}, y={pose[1]}, z={pose[2]}, rx={pose[3]}, ry={pose[4]}, rz={pose[5]}")
            return False
            

    def publish_trigger(self):
        '''publishes the trigger message. This is used to determine if the camera should publish an image'''
        msg = Bool()
        msg.data = self.home

        if self.home == True:
            self.trigger_publisher.publish(msg)
            self.get_logger().info('trigger publish = true')
        elif self.home == False:
            self.trigger_publisher.publish(msg)
            self.get_logger().info('trigger publish = false')


    def process_fruit_positions(self, msg):
        '''move to home position and process the first fruit in the list'''
        try:
            self.get_logger().info('Processing Fruit Positions')
            #move in joint space to home position
            self.rtde_c.moveJ([0, -4.136, 2.53, -2.79, -1.57, 0], speed=self.spd*2, acceleration=self.accel)
            #open gripper
            self.rtde_io.setConfigurableDigitalOut(7, 0)
            self.home = True

            #clear arrays
            position = []
            rotation = []

            #load fruit positions
            fruit_positions = json.loads(msg.data)

            #get first fruit
            if fruit_positions:   
                position = fruit_positions[0].get("position")
                rotation = fruit_positions[0].get("rotation")

            #if there are fruit
            if len(position) == 3 and rotation:
                #stop publishing camera messages
                self.home = False
                self.publish_trigger()      

                #pick fruit
                self.pick_fruit(position, rotation)
            else:
                self.publish_trigger()


        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error decoding JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing targets: {e}")

    def pick_fruit(self, position, rotation):
        """Pick fruit and return"""
        try:
            #set position to variables
            x, y, z = position
            rz = rotation
            rz_flipped = rotation - 3.14

            # Define waypoint 0.15m below target
            waypoint_z = z - 0.15

            #check if waypoint is below working space (May be redundant but change as needed)
            if waypoint_z < 0:
                self.get_logger().warning("Waypoint z is below the robot's working space. Skipping target")
                return

            #set desired pose with a normal gripper orientation and reversed gripper orientation
            waypoint_pose = [x, y, waypoint_z, 0.0, 0.0, rz]
            waypoint_pose_flipped = [x, y, waypoint_z, 0.0, 0.0, rz_flipped]

            #move to pose
            if self.move_to_pose_and_log(waypoint_pose, "Moved to waypoint"):
                #set next point
                target_pose = [x, y, z, 0.0, 0.0, rz]
                #move to pose
                if self.move_to_pose_and_log(target_pose, "Moved to target"):
                    # Close gripper
                    self.rtde_io.setConfigurableDigitalOut(7, 1)
                    #set next pose
                    tilted_pose = self.rotate_local_rx()
                    #move to pose
                    if self.move_to_pose_and_log(tilted_pose, "Picking"):
                        #set next pose
                        waypoint_tilted_pose = [x, y, waypoint_z, *tilted_pose[3:]]
                        #move to pose
                        if self.move_to_pose_and_log(waypoint_tilted_pose, "Picked fruit"):
                            #move in joint space to dropoff position
                            self.rtde_c.moveJ([0, -4.136, 2.53, -2.7, -0.7, 0.26], speed=self.spd*2, acceleration=self.accel)
                            # Open gripper
                            self.rtde_io.setConfigurableDigitalOut(7, 0)

            #follow same format as above but with reversed gripper
            #any changes to first if statement must be changed here aswell
            elif self.move_to_pose_and_log(waypoint_pose_flipped, "Moved to waypoint"):

                target_pose = [x, y, z, 0.0, 0.0, rz_flipped]

                if self.move_to_pose_and_log(target_pose, "Moved to target"):
                    self.rtde_io.setConfigurableDigitalOut(7, 1)
                    tilted_pose = self.rotate_local_rx()

                    if self.move_to_pose_and_log(tilted_pose, "Picking"):
                        waypoint_tilted_pose = [x, y, waypoint_z, *tilted_pose[3:]]

                        if self.move_to_pose_and_log(waypoint_tilted_pose, "Picked fruit"):                                
                            self.rtde_c.moveJ([0, -4.136, 2.53, -2.7, -0.7, 0.26], speed=self.spd*2, acceleration=self.accel)
                            self.rtde_io.setConfigurableDigitalOut(7, 0)

        except Exception as e:
            self.get_logger().error(f"Error Picking Fruit: {e}")

    def rotate_local_rx(self):
        """Rotates the TCP around its local rx axis."""
        current_pose = self.rtde_r.getActualTCPPose()
        joints = self.rtde_c.getInverseKinematics(current_pose)

        #chnage this number to change the picking angle
        tilt_angle = 1 # 1 is ~60 degrees

        position = np.array(current_pose[:3])
        orientation = np.array(current_pose[3:])

        #this will cause the TCP to always rotate towards the robot
        #if the mounting of the robot is changed this will need to be adjusted to better represent the optimal rotation
        if joints[5] < 0:
            tilt_angle *= -1

        #does matrix math to rotate tcp in plane with the gripper
        base_rotation_matrix = R.from_rotvec(orientation).as_matrix()
        local_rotation_matrix = R.from_rotvec([0, tilt_angle, 0]).as_matrix()
        new_rotation_matrix = base_rotation_matrix @ local_rotation_matrix
        new_orientation = R.from_matrix(new_rotation_matrix).as_rotvec()
        return list(position) + list(new_orientation)

    def stop_robot(self):
        """Stops the robot safely."""
        #stop any curent actions on the robot
        self.rtde_c.stopScript()
        #disconnect from rtde ports
        self.rtde_c.disconnect()
        self.rtde_io.disconnect()
        self.rtde_r.disconnect()
        self.get_logger().info("Robot safely stopped")


def main(args=None):
    rclpy.init(args=args)
    node = FruitPickerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()