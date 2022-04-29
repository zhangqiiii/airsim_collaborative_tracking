"""
test python environment
"""
import threading

import airsim
import math
import time

camera_name = "high_res"


class flying(threading.Thread):  # 继承父类threading.Thread
    def __init__(self, name, client_: airsim.MultirotorClient):
        threading.Thread.__init__(self)
        self.name = name
        self.client = client_

    def run(self):
        self.client.moveToZAsync(-20, 5).join()
        self.client.rotateToYawAsync(90).join()
        self.client.moveToPositionAsync(10, 10, -20, 2)
        time.sleep(2)
        self.client.moveToPositionAsync(10, 50, -20, 2).join()
        time.sleep(100)

        # 内部结束线程的函数(threading.Thread).exit()


class print_state(threading.Thread):
    def __init__(self, name, client_: airsim.MultirotorClient):
        threading.Thread.__init__(self)
        self.name = name
        self.client = client_

    def run(self):
        while True:
            drone_state = client.getMultirotorState()
            print(drone_state.kinematics_estimated.position)


def get_rotate_degree(client):
    drone_state = client.getMultirotorState()
    # 位姿是四元数表示的
    orient = drone_state.kinematics_estimated.orientation
    # 四元数转化为欧拉角
    eular = airsim.to_eularian_angles(orient)
    print(eular)
    print(eular[2] / 2.0 / math.pi * 360)


# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()

# get control
client.enableApiControl(True)

# unlock
client.armDisarm(True)

# Async methods returns Future. Call join() to wait for task to complete.
client.takeoffAsync().join()

client.moveToZAsync(-20, 5).join()

# dt = airsim.DrivetrainType.ForwardOnly
dt = airsim.DrivetrainType.ForwardOnly
yawmode = airsim.YawMode(is_rate=False, yaw_or_rate=0)
# client.moveToPositionAsync(40, 40, -20, 5, 300,
#                            drivetrain=dt, yaw_mode=yawmode,
#                            lookahead=1, adaptive_lookahead=-1).join()

client.rotateToYawAsync(60).join()
client.moveByVelocityAsync(5, 0, 0, 15,
                           drivetrain=dt,
                           yaw_mode=yawmode).join()
print("move to 20 0 0")
res = client.moveToZAsync(-20, 5)
while True:
    if res.result is not None:
        print(res.result)
# print(res.result)
print("end")

# print(airsim.to_eularian_angles(client.simGetCameraInfo(camera_name).pose.orientation))
# client.rotateToYawAsync(90).join()
# print(airsim.to_eularian_angles(client.simGetCameraInfo(camera_name).pose.orientation))
# client.rotateToYawAsync(30).join()
# print(airsim.to_eularian_angles(client.simGetCameraInfo(camera_name).pose.orientation))

# client.simSetCameraPose(camera_name, airsim.Pose(orientation_val=airsim.to_quaternion(-1,0,0)))
# client.moveToPositionAsync(10, 10, -20, 2)
# time.sleep(2)
# drone_state = client.getMultirotorState()
# print(drone_state.kinematics_estimated.position)
# client.moveToPositionAsync(10, 50, -20, 2)
#
# drone_state = client.getMultirotorState()
# print(drone_state.kinematics_estimated.position)
# time.sleep(20)

# drone_state = client.getMultirotorState()
# print(drone_state.kinematics_estimated.position)

# 创建新线程
# thread1 = flying("Thread-1", client)
# thread2 = print_state("Thread-2", client)

# 开启线程
# thread1.start()
# thread2.start()

# f = client.goHomeAsync()
# client.goHomeAsync().join()
# print("landing")
# client.landAsync().join()

# lock
print("locking")
client.armDisarm(False)

# release control
client.enableApiControl(False)
