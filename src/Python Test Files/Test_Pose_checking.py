import rtde_control, rtde_receive, json

class RobotArm:
    def __init__(self, robot_ip):
        self.robot_ip = robot_ip

        #Initialise RTDE Control interfaces
        self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
        
    def Check_Pose(self, pose):
        try:

            if self.rtde_c.isPoseWithinSafetyLimits(pose) == True:
                print("reachable")
            else:
                print("unreachable")
        except Exception as e:
            print(f"Error Processing Pose: {e}")

    def stop_robot(self):

        self.rtde_c.stopScript()
        print("Robot Safely Stopped")

if __name__ == '__main__':
    x = -0.473
    y = 0.178
    z = 0.579
    rx = 0.0
    ry = 0.0
    rz = 1.57

    example_pose = [x,y,z,rx,ry,rz]
    #initiialize robot
    robot_ip = "192.168.131.8"
    robot = RobotArm(robot_ip)

    robot.Check_Pose(example_pose)

    robot.stop_robot()
