import rtde_control, rtde_receive, rtde_io, json, rclpy
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from std_msgs.msg import String


class FruitPickerNode(Node):
    def __init__(self):
        super().__init__('fruit_picker')
        self.declare_parameter('robot_ip', '192.168.131.8')
        self.robot_ip = self.get_parameter('robot_ip').value
        self.spd = 0.2
        self.accel = 0.1

        #Initialise RTDE Control and Recieve interfaces
        self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)
        self.rtde_io = rtde_io.RTDEIOInterface(self.robot_ip)

        #subscribe to gripper_orientation topic
        self.subscription = self.create_subscription(String, '/gripper_orientation', self.process_fruit_positions, 10)
        self.get_logger().info("Fruit picker Initialised")
        
    def process_fruit_positions(self, msg):
        
        try:
            #parse json data
            fruit_positions = json.loads(msg.data)

            for fruit in fruit_positions:
                
                position = fruit.get("position")
                rotation = fruit.get("rotation")

                if not position or len(position) != 3 or rotation is None:
                    self.get_logger().warning("Invalid fruit data format. Skipping Entry")
                    continue

                x, y, z = position
                rz = rotation

                self.get_logger().info(f"Processing Target: x={x}, y={y}, z={z}, rz={rz}")

                #define waypoint 0.1m below target
                waypoint_z = z - 0.15
                if waypoint_z < 0: #edit for a minimum working height
                    self.get_logger().warning("waypoint z is below the robots working space. Skipping Target")
                    continue                
                
                if self.rtde_c.isPoseWithinSafetyLimits([x,y,waypoint_z,0.0,0.0,rz]) == True:
                    #move to waypoint
                    self.rtde_c.moveL([x, y, waypoint_z, 0.0, 0.0, rz], speed=self.spd, acceleration=self.accel)
                    self.get_logger().info(f"moved to wapoint at x={x}, y={y}, z={waypoint_z}")

                    if self.rtde_c.isPoseWithinSafetyLimits([x,y,z,0.0,0.0,rz]) == True:
                        #move to target vertically
                        self.rtde_c.moveL([x, y, z, 0.0, 0.0, rz], speed=self.spd, acceleration=self.accel)
                        self.get_logger().info(f"moved to wapoint at x={x}, y={y}, z={z} with orientation {rz}")

                        self.rtde_io.setConfigurableDigitalOut(7,1)

                        #tilt gripper around local rx
                        current_pose = self.rtde_r.getActualTCPPose()
                        tilt_rx = 1.309 #75 degrees
                        tilted_pose = self.rotate_local_rx(current_pose, tilt_rx)

                        if self.rtde_c.isPoseWithinSafetyLimits(tilted_pose) == True:
                            #Tilt head around TCP x-axis for optimal picking
                            self.rtde_c.moveL(tilted_pose, speed=self.spd, acceleration=self.accel)
                            self.get_logger().info(f"picking at x={x}, y={y}, z={z} with orientation {rz}")

                            if self.rtde_c.isPoseWithinSafetyLimits([x,y,waypoint_z, *tilted_pose[3:]]) == True:
                                #Move head down to intermediary waypoint at new angle
                                self.rtde_c.moveL([x, y, waypoint_z, *tilted_pose[3:]], speed=self.spd, acceleration=self.accel)
                                self.get_logger().info(f"picked fruit at x={x}, y={y}, z={z} with orientation {rz}")
                                self.rtde_io.setConfigurableDigitalOut(7,0)
                            else:
                                self.get_logger().warn(f"Pose Unreachable: {x}, {y}, {z}, 0.0, 0.0, {rz}")
                                self.rtde_io.setConfigurableDigitalOut(7,0)
                        else:
                            self.get_logger().warn(f"Pose Unreachable: {x}, {y}, {z}, 0.0, 0.0, {rz}")
                            self.rtde_io.setConfigurableDigitalOut(7,0)
                    else:
                        self.get_logger().warn(f"Pose Unreachable: {x}, {y}, {z}, 0.0, 0.0, {rz}")
                        self.rtde_io.setConfigurableDigitalOut(7,0)
                else:
                    self.get_logger().warn(f"Pose Unreachable: {x}, {y}, {z}, 0.0, 0.0, {rz}")
                    self.rtde_io.setConfigurableDigitalOut(7,0)
                    

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error decoding json: {e}")
        except Exception as e:
            self.get_logger().error(f"Error Processing Targets: {e}")

    def rotate_local_rx(self, base_pose, local_rx_angle):
        #rotate the tool about the local rx without changing rz?

        #extract the position and orientation
        position = np.array(base_pose[:3])
        orientation = np.array(base_pose[3:])

        #convert orientation to a rotation matrix
        base_rotation_matrix = R.from_rotvec(orientation).as_matrix()

        #define local rotation around axis
        local_rotation_matrix = R.from_rotvec([0, local_rx_angle, 0]).as_matrix()

        #compute local rotation matrix in base frame
        new_rotation_matrix = base_rotation_matrix @ local_rotation_matrix

        #convert to orientation vector
        new_orientation = R.from_matrix(new_rotation_matrix).as_rotvec()

        #return the new pose
        return list(position) + list(new_orientation)

    def stop_robot(self):
        self.rtde_c.stopScript()
        self.rtde_io.stopScript()
        self.rtde_r.stopScript()
        self.get_logger().info("Robot Safely Stopped")

def main(args=None):
    rclpy.init(args=args)
    node = FruitPickerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("node stopper by user")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
