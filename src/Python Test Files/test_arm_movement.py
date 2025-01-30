import rtde_control, rtde_receive, json

class RobotArm:
    def __init__(self, robot_ip):
        self.robot_ip = robot_ip

        #Initialise RTDE Control and Recieve interfaces
        self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)
        
    def process_fruit_positions(self, json_data):
        try:
            #parse json data
            fruit_positions = json.loads(json_data)

            for fruit in fruit_positions:
                position = fruit.get("position")
                rotation = fruit.get("rotation")

                if not position or len(position) != 3 or rotation is None:
                    print("Invalid fruit data format. Skipping Entry")
                    continue

                x, y, z = position
                rz = rotation

                print(f"Processing Target: x={x}, y={y}, z={z}, rz={rz}")

                #define waypoint 0.1m below target
                waypoint_z = z - 0.1
                approach_rx = 0.0
                picking__rx = -0.75

                if waypoint_z < 0: #edit for a minimum working height
                    print("waypoint z is below the robots working space. Skipping Target")
                    continue

                #orient TCP for vertical approach (rx,ry) = (0,0) change rz for correct orientation
                approach_orientation = [approach_rx, 0.0, rz]
                picking_orientation = [picking__rx, 0.0, rz]

                #move to waypoint
                self.rtde_c.moveL([x, y, waypoint_z, *approach_orientation], speed=0.5, acceleration=0.3)
                print(f"moved to wapoint at x={x}, y={y}, z={waypoint_z}")

                #move to target vertically
                self.rtde_c.moveL([x, y, z, *approach_orientation], speed=0.5, acceleration=0.3)
                print(f"moved to wapoint at x={x}, y={y}, z={z} with orientation {rz}")

                #Tilt head around TCP x-axis for optimal picking
                self.rtde_c.moveL([x, y, z, *picking_orientation], speed=0.5, acceleration=0.3)
                print(f"picking at x={x}, y={y}, z={z} with orientation {rz}")

                #Move head down to intermediary waypoint at new angle
                self.rtde_c.moveL([x, y, waypoint_z, *picking_orientation], speed=0.5, acceleration=0.3)
                print(f"picked fruit at x={x}, y={y}, z={z} with orientation {rz}")

        except json.JSONDecodeError as e:
            print(f"Error decoding json: {e}")
        except Exception as e:
            print(f"Error Processing Targets: {e}")

    def stop_robot(self):
        self.rtde_c.stopScript()
        self.rtde_r.stopScript()
        print("Robot Safely Stopped")

if __name__ == '__main__':
    #example JSON string
    example_data = '[{"position": [-0.5, 0.0, 0.9], "rotation": 0.0}, {"position": [-0.6, -0.1, 0.8], "rotation": 1.57}, {"position": [-0.5, 0.2, 0.7], "rotation": -0.8}]'

    #initiialize robot
    robot_ip = "192.168.131.8"
    robot = RobotArm(robot_ip)

    robot.process_fruit_positions(example_data)

    robot.stop_robot()
