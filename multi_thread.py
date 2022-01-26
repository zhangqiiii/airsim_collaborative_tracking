import pprint
import threading
import airsim
import math
import time
from tracking import rotate_2vector, det_coord, get_2d_gaussian_model
from param_setting import *


class droneControl(threading.Thread):
    """
    单个无人机的控制
    """

    def __init__(self, vehicle_name, client: airsim.MultirotorClient):
        threading.Thread.__init__(self)
        self.vehicle_name = vehicle_name
        self.client = client

    def run(self):
        camera_pose = airsim.Pose(position_val=airsim.Vector3r(0.5, 0.0, 0.1),
                                  orientation_val=airsim.to_quaternion(-pi / 4., 0, 0))
        self.client.simSetCameraPose(camera_name, camera_pose, self.vehicle_name)
        self.client.armDisarm(True)
        print(self.vehicle_name + ": takeoff...")
        self.client.takeoffAsync().join()

        current_height = 20.0
        self.client.moveToZAsync(-1.0 * current_height, 5).join()

        # 每次升高10m巡逻
        add_height = 10.0
        # 每一圈用时10s
        round_time = 10.0
        self.client.simSetDetectionFilterRadius(camera_name, image_type, 200 * 100)  # in [cm]
        self.client.simAddDetectionFilterMeshName(camera_name, image_type, "ThirdPersonCharacter*")
        gaussian_weight = get_2d_gaussian_model()

        find_obj = False
        # 逐步升高找寻目标
        objs = []
        while current_height < 100.0 and not find_obj:
            # if rotate_with_camera(client):
            #     find_obj = True
            #     break
            start_time = time.time()
            # 以一定角速度运动到某个高度
            self.client.rotateByYawRateAsync(360 / round_time, round_time, self.vehicle_name)
            while time.time() - start_time < round_time:
                objs = self.client.simGetDetections(camera_name, image_type)
                if objs:
                    self.client.cancelLastTask(self.vehicle_name)
                    find_obj = True
                    print(self.vehicle_name +": have found")
                    break

            current_height += add_height
            self.client.moveToZAsync(-1.0 * current_height, 5)
        # 没有检测到目标
        if not find_obj:
            print(self.vehicle_name + ": Not find target")
        # 检测到目标
        else:
            print(self.vehicle_name + ": Closing to target")

            while True:
                objs = self.client.simGetDetections(camera_name, image_type)
                # 暂时只考虑一个目标的情况
                if len(objs) == 0:
                    continue
                res = det_coord(objs)
                direct_vec = (res[0], res[1])
                # print("方向向量为：%s" % pprint.pformat(direct_vec))

                # 获取当前无人机的位置信息
                drone_state = self.client.getMultirotorState(self.vehicle_name)
                angle = airsim.to_eularian_angles(drone_state.kinematics_estimated.orientation)
                # 摆动太大，等待稳定
                thr = math.radians(10)
                if angle[0] > thr or angle[1] > thr:
                    print(self.vehicle_name + ": 摆动太大")
                    # client.hoverAsync(self.vehicle_name).join()
                    continue

                # 位姿是四元数表示的
                orient = drone_state.kinematics_estimated.orientation
                euler = airsim.to_eularian_angles(orient)

                # 旋转角度前面要乘以 -1
                new_vec = rotate_2vector(-1 * euler[2], direct_vec)

                weighted_speed = max_speed * gaussian_weight[res[4], res[3]]
                print(self.vehicle_name + ": " + weighted_speed)
                self.client.moveToPositionAsync(new_vec[1] + drone_state.kinematics_estimated.position.x_val,
                                                new_vec[0] + drone_state.kinematics_estimated.position.x_val,
                                                -1 * current_height, weighted_speed,
                                                vehicle_name=self.vehicle_name)


if __name__ == "__main__":
    vehicle_name1 = "Drone1"
    client1 = airsim.MultirotorClient()
    client1.confirmConnection()
    client1.enableApiControl(True, vehicle_name)
    drone1 = droneControl(vehicle_name1, client1)

    vehicle_name2 = "Drone2"
    client2 = airsim.MultirotorClient()
    client2.confirmConnection()
    client2.enableApiControl(True, vehicle_name2)
    drone2 = droneControl(vehicle_name2, client2)

    drone1.start()
    print("drone1 start")
    drone2.start()
    print("drone2 start")
