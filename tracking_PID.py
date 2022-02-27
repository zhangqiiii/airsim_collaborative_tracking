import pprint
import airsim
import cv2
import numpy as np
import time

# 导入全局参数
from param_setting import *
from PIDcontroller import PIDController

vehicle_name = "Drone1"

def rotate_2vector(theta, vec):
    """
    通过二维旋转矩阵计算旋转后的二维坐标
    :param theta: 逆时针旋转的度数，单位是弧度
    :param vec: [tuple|list|array]待旋转的二维坐标
    :return: [tuple]旋转后的二维坐标
    """
    # 旋转矩阵
    rotate_mat = np.mat([[math.cos(theta), -1.0 * math.sin(theta)],
                         [math.sin(theta), math.cos(theta)]])
    new_vec = rotate_mat * np.matrix(vec).T

    return new_vec[0, 0], new_vec[1, 0]


def det_coord(objs):
    """
    返回目标的检测结果
    :param objs: simGetDetections的检测结果
    :return: 5个值, 方向向量x, y, 模长, 图像位置的坐标x, y
    """
    obj = objs[0]
    center_x = obj.box2D.min.x_val + obj.box2D.max.x_val
    center_x /= 2.
    center_y = obj.box2D.min.y_val + obj.box2D.max.y_val
    center_y /= 2.

    v_dir_x = center_x - camera_width / 2.
    v_dir_y = camera_height / 2. - center_y
    norm = math.sqrt(v_dir_x * v_dir_x + v_dir_y * v_dir_y)

    return v_dir_x, v_dir_y, norm, int(center_x), int(center_y)


def main():
    client = airsim.MultirotorClient()
    client.confirmConnection()

    client.enableApiControl(True, vehicle_name)
    print("SetCamera...")
    camera_pose = airsim.Pose(position_val=airsim.Vector3r(0.5, 0.0, 0.1),
                              orientation_val=airsim.to_quaternion(-pi / 4., 0, 0))
    client.simSetCameraPose(camera_name, camera_pose, vehicle_name)

    print("Taking off...")
    client.armDisarm(True)
    client.takeoffAsync().join()

    current_height = 20.0
    print("Move to 20m...")
    client.moveToZAsync(-1.0 * current_height, 5).join()

    # 每次升高10m巡逻
    add_height = 10.0
    # 每一圈用时10s
    round_time = 10.0
    client.simSetDetectionFilterRadius(camera_name, image_type, 200 * 100)  # in [cm]
    client.simAddDetectionFilterMeshName(camera_name, image_type, "ThirdPersonCharacter*")

    find_obj = False
    # 逐步升高找寻目标
    objs = []
    while current_height < 100.0 and not find_obj:
        start_time = time.time()
        # 以一定角速度运动到某个高度
        client.rotateByYawRateAsync(360 / round_time, round_time, vehicle_name)
        while time.time() - start_time < round_time:
            objs = client.simGetDetections(camera_name, image_type)
            if objs:
                client.cancelLastTask(vehicle_name)
                find_obj = True
                print("have found")
                break

        current_height += add_height
        client.moveToZAsync(-1.0 * current_height, 5)
    # 没有检测到目标
    if not find_obj:
        print("Not find target")
    # 检测到目标
    else:
        print("Closing to target")
        # 分别创建两个 PID 来控制 偏航 和 前向速度
        speed_ctrl = PIDController(0.001, 0.01, 0.01, 15)
        yaw_ctrl = PIDController(0.08, 0.01, 0.05, math.radians(8))
        while True:
            objs = client.simGetDetections(camera_name, image_type)
            # 暂时只考虑一个目标的情况
            if len(objs) == 0:
                continue
            # 获取当前无人机的位置信息
            drone_state = client.getMultirotorState(vehicle_name)
            angle = airsim.to_eularian_angles(drone_state.kinematics_estimated.orientation)

            res = det_coord(objs)
            cur_yaw = math.atan2(res[1], res[0])

            # 偏航增量
            yaw_add = yaw_ctrl.getOutput(pi / 2 - cur_yaw)
            # print("无人机偏航" + str(math.degrees(angle[2])))
            # print("偏航 " + str(math.degrees(yaw_add)))

            cur_speed = speed_ctrl.getOutput(res[2])
            vy = cur_speed * math.sin(cur_yaw)
            vx = cur_speed * math.cos(cur_yaw)
            vy, vx = rotate_2vector(-angle[2], (vx, vy))

            # 判断是否需要偏航
            if 0 < cur_yaw < pi:
                yaw_mode = airsim.YawMode(False, math.degrees(angle[2] + yaw_add))
            else:
                yaw_mode = airsim.YawMode(True, 0)

            # 目标在中间区域后适当降低高度
            if cur_speed < 2:
                current_height = 15.0

            # 跟随
            client.moveByVelocityZAsync(vx, vy, -current_height, 2,
                                        airsim.DrivetrainType.MaxDegreeOfFreedom,
                                        yaw_mode,
                                        vehicle_name=vehicle_name)

    client.landAsync().join()
    client.reset()
    client.armDisarm(False)
    client.enableApiControl(False)


if __name__ == "__main__":
    main()
