"""
test python environment
"""
import threading

import airsim
import math
import time

from t_util import droneLidar
from tracking_PID import vehicle_name

camera_name = "high_res"


class flying(threading.Thread):  # 继承父类threading.Thread
    def __init__(self, name, client_: airsim.MultirotorClient):
        threading.Thread.__init__(self)
        self.name = name
        self.client = client_

    def run(self):
        # self.client.moveToZAsync(-30, 3).join()
        # time.sleep(2)
        # self.client.rotateToYawAsync(90).join()
        # self.client.moveToPositionAsync(20, 20, -20, 2).join()
        print("wancheng")

        # 内部结束线程的函数(threading.Thread).exit()


class print_state(threading.Thread):
    def __init__(self, name, client_: airsim.MultirotorClient):
        threading.Thread.__init__(self)
        self.name = name
        self.client = client_

    def run(self):
        time.sleep(5)
        print(0)
        f = self.client.moveToPositionAsync(0, 0, -20, 2)
        print("-1")
        print("33")

        while True:
            # drone_state = self.client.getMultirotorState()
            # print(drone_state.kinematics_estimated.position)
            time.sleep(1)


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
# t1 = flying("ee", client)
# t1.start()
# client__ = airsim.MultirotorClient()
# t2 = print_state("fff", client__)
# t2.start()
# from t_util import moveToPositionWithLidar
#
# moveToPositionWithLidar(client, 8, -40, -10, 5, "Drone1")
lidar_ctrl = droneLidar(client, vehicle_name)
yaw = 0
# client.moveToPositionAsync(0, 10, -2, 2)
# f = open("./test_sensing", "w")
for i in range(100):
    # client.rotateToYawAsync(yaw)
    # yaw += 15
    time.sleep(0.1)
    a = lidar_ctrl.sensing()

# current_height = 5.0
# while True:
#     client.moveByVelocityZAsync(2, 2, -current_height, 4).join()
#     current_height += 2.0

# # dt = airsim.DrivetrainType.ForwardOnly
# dt = airsim.DrivetrainType.ForwardOnly
# yawmode = airsim.YawMode(is_rate=False, yaw_or_rate=0)
# # client.moveToPositionAsync(40, 40, -20, 5, 300,
# #                            drivetrain=dt, yaw_mode=yawmode,
# #                            lookahead=1, adaptive_lookahead=-1).join()
#
# f = client.rotateToYawAsync(60)
#
# print(client.getMultirotorState())
#
# client.moveToPositionAsync(0, 0, -30, 5).join()
# print("ok")
#
# # num = 0
# # while True:
# #     client.moveByVelocityAsync(5, -5, 0, 15,
# #                             drivetrain=dt,
# #                             yaw_mode=yawmode)
# #     num += 1
# #     print("num ", num)
#
# print("move to 20 0 0")
# res = client.moveToZAsync(-20, 5)
# while True:
#     if res.result is not None:
#         print(res.result)
# # print(res.result)
# print("end")

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
# print("locking")
# client.armDisarm(False)
#
# release control
# client.enableApiControl(False)
