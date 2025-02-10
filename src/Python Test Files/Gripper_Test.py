import rtde_control, rtde_receive, rtde_io, time

'''close and open the gripper'''

class RobotArm:
    def __init__(self, robot_ip):
        self.robot_ip = robot_ip

        #Initialise RTDE Control interfaces
        self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
        self.rtde_io = rtde_io.RTDEIOInterface(self.robot_ip)
        
    def Test_Grip(self):
        try:                 
                self.rtde_io.setConfigurableDigitalOut(7,1)
                time.sleep(0.75)
                self.rtde_io.setConfigurableDigitalOut(7,0)


        except Exception as e:
            print(f"Error Processing Pose: {e}")

    def stop_robot(self):
        self.rtde_c.stopScript()
        self.rtde_c.disconnect()        
        self.rtde_io.disconnect()
        print("Robot Safely Stopped")

if __name__ == '__main__':
    robot_ip = "192.168.131.8"
    robot = RobotArm(robot_ip)
    time.sleep(1)
    robot.Test_Grip()

    robot.stop_robot()
