import pprint
import airsim
import cv2
import numpy as np
import time

# 导入全局参数
from param_setting import *


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


def get_2d_gaussian_model():
    """
    返回一个二维高斯核，用来调节在不同视野位置下无人机的速度
    :return: 返回值是速度的权重图，一次计算，重复使用
    """
    upsample = 12
    h_sigma = 6
    w_sigma = h_sigma * camera_width / camera_height
    row = cv2.getGaussianKernel(int(camera_height / upsample), h_sigma)
    col = cv2.getGaussianKernel(int(camera_width / upsample), int(w_sigma)).T
    model = row * col
    max_value = np.max(model)
    model = model / max_value
    model = cv2.resize(model, (camera_width, camera_height))
    # cv2.imshow("e", model)
    # cv2.waitKey()
    return 1 - model


def rotate_with_camera(client: airsim.MultirotorClient):
    yaw = 0
    position = client.getMultirotorState(vehicle_name).kinematics_estimated.position
    while yaw < 360.0:
        drone_state = client.getMultirotorState(vehicle_name).kinematics_estimated
        curr_position = drone_state.position
        dist = math.pow(curr_position.x_val - position.x_val, 2) + \
               math.pow(curr_position.y_val - position.y_val, 2) + \
               math.pow(curr_position.z_val - position.z_val, 2)
        # 不是悬停状态
        if dist >= 3:
            break
        yaw += 1
        client.rotateToYawAsync(yaw, vehicle_name=vehicle_name).join()
        camera_pose = client.simGetCameraInfo(camera_name, vehicle_name).pose
        euler = airsim.to_eularian_angles(camera_pose.orientation)
        new_camera_pose = airsim.Pose(airsim.Vector3r(0.5, 0, 0.1),
                                      airsim.to_quaternion(euler[0], euler[1], math.radians(yaw)))
        client.simSetCameraPose(camera_name, new_camera_pose, vehicle_name)

        objs = client.simGetDetections(camera_name, image_type)
        if objs:
            return True

    return False

# def stabilize_camera(drone_euler, client: airsim.MultirotorClient):
#     """
#     由于无人机会摆动，这里给相机模拟一个云台的效果用来稳定相机
#     :param drone_euler: 无人机的欧拉角
#     :param client: 无人机控制客户端
#     :return:
#     """
#     camera_info = client.simGetCameraInfo(camera_name, vehicle_name)
#     camera_euler = airsim.to_eularian_angles(camera_info.pose.orientation)
#     new_camera_euler = np.array(camera_euler) - np.array([drone_euler[0], 0, 0])
#     # 创建位姿，原来的 position 和新的 orientation
#     """相机的Pose是相对于NED坐标而言的"""
#     camera_pose = airsim.Pose(position_val=camera_info.pose.position,
#                               orientation_val=airsim.to_quaternion(new_camera_euler[0],
#                                                                    new_camera_euler[1], new_camera_euler[2]))
#     client.simSetCameraPose(camera_name, camera_pose, vehicle_name)


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

        while True:
            objs = client.simGetDetections(camera_name, image_type)
            # 暂时只考虑一个目标的情况
            if len(objs) == 0:
                continue
            res = det_coord(objs)
            direct_vec = (res[0], res[1])
            print("方向向量为：%s" % pprint.pformat(direct_vec))
            # yaw = math.atan2(v_dir_y, v_dir_x)
            # yaw = yaw / (2*pi) * 360

            # 获取当前无人机的位置信息
            drone_state = client.getMultirotorState(vehicle_name)
            angle = airsim.to_eularian_angles(drone_state.kinematics_estimated.orientation)
            # 摆动太大，等待稳定
            thr = math.radians(10)
            if angle[0] > thr or angle[1] > thr:
                print("摆动太大")
                # client.hoverAsync(vehicle_name).join()
                continue

            # 位姿是四元数表示的
            orient = drone_state.kinematics_estimated.orientation
            # 四元数转化为欧拉角（pitch，roll，yaw）
            euler = airsim.to_eularian_angles(orient)

            # 旋转角度前面要乘以 -1
            new_vec = rotate_2vector(-1 * euler[2], direct_vec)

            weighted_speed = max_speed * gaussian_weight[res[4], res[3]]
            print(weighted_speed)
            # 这里 vx,vy,vz都是无人机坐标NED下的位置
            client.moveToPositionAsync(new_vec[1] + drone_state.kinematics_estimated.position.x_val,
                                       new_vec[0] + drone_state.kinematics_estimated.position.x_val,
                                       -1 * current_height, weighted_speed,
                                       vehicle_name=vehicle_name)

    client.landAsync().join()
    client.reset()
    client.armDisarm(False)
    client.enableApiControl(False)


if __name__ == "__main__":
    main()
    # get_2d_gaussian_model()
