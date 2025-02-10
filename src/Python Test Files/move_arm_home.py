import rtde_control, rtde_receive


'''move arm to point in either linear or joint spacae
just uncomment to choose which'''

class RobotArm:
    def __init__(self, robot_ip):
        self.robot_ip = robot_ip

        #Initialise RTDE Control and Recieve interfaces
        self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)

        
        
    def process_fruit_positions(self, waypoint):
        try:
            
            #self.rtde_c.moveL(waypoin)
            self.rtde_c.moveJ([0, -4.136, 2.53, -2.7, -0.7, 0])

        except Exception as e:
            print(f"Error Moving Home: {e}")

    def stop_robot(self):
        self.rtde_c.stopScript()
        print("Robot Safely Stopped")

if __name__ == '__main__':

    #initiialize robot
    robot_ip = "192.168.131.8"
    robot = RobotArm(robot_ip)

    #home coords (x    y    z    rx ry rz)
    home_point = (-0.5, 0.2, 0.2, 0, 0, 0)

    robot.process_fruit_positions(home_point)

    robot.stop_robot()
