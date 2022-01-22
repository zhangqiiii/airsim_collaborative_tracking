import pprint

import airsim
import cv2
import numpy
import numpy as np
import math
import time

pi = math.pi
camera_width = 600
camera_height = 400


def rotate_2vector(theta, vec):
    # 旋转矩阵
    rotate_mat = np.mat([[math.cos(theta), -1.0 * math.sin(theta)],
                         [math.sin(theta), math.cos(theta)]])
    new_vec = rotate_mat * np.matrix(vec).T

    return new_vec[0, 0], new_vec[1, 0]


def det_coord(objs):
    obj = objs[0]
    center_x = obj.box2D.min.x_val + obj.box2D.max.x_val
    center_x /= 2.
    center_y = obj.box2D.min.y_val + obj.box2D.max.y_val
    center_y /= 2.

    v_dir_x = center_x - camera_width / 2.
    v_dir_y = camera_height / 2. - center_y
    norm = math.sqrt(v_dir_x * v_dir_x + v_dir_y * v_dir_y)

    return v_dir_x, v_dir_y, norm, int(center_x), int(center_y)

# def moveToPositionWithAcc(client: airsim.MultirotorClient, max_v, acc, x, y, z, vehicle_name):
#     while True:
#         state = client.getMultirotorState(vehicle_name)
#         state.kinematics_estimated.linear_acceleration
#     client.moveToPositionAsync(x, y, z,)


def get_2d_gaussian_model():
    upsample = 12
    h_sigma = 8
    w_sigma = 8 * camera_width / camera_height
    row = cv2.getGaussianKernel(int(camera_height / upsample), h_sigma)
    col = cv2.getGaussianKernel(int(camera_width / upsample), int(w_sigma)).T
    model = row * col
    max_value = np.max(model)
    model = model / max_value
    model = cv2.resize(model, (camera_width, camera_height))
    print(model.shape)
    input()
    return 1 - model


def main():
    vehicle_name = "Drone1"
    camera_name = "high_res"
    client = airsim.MultirotorClient()
    client.confirmConnection()

    client.enableApiControl(True, vehicle_name)
    print("SetCamera...")
    camera_pose = airsim.Pose(orientation_val=airsim.to_quaternion(-45, 0, 0))
    client.simSetCameraPose(camera_name, camera_pose, vehicle_name)

    print("Taking off...")
    client.armDisarm(True)
    client.takeoffAsync().join()

    current_height = 20.0
    print("Move to 20m...")
    client.moveToZAsync(-1.0 * current_height, 5).join()

    # 获取相机参数
    # print("GetCameraInfo...")
    # camera_info = client.simGetCameraInfo(camera_name, vehicle_name)
    # print("info: %s" % pprint.pformat(camera_info))

    # print("Press any key to continue")
    # airsim.wait_key()

    # 每次升高10m巡逻
    add_height = 10.0
    # 每一圈用时10s
    round_time = 10.0
    image_type = airsim.ImageType.Scene
    client.simSetDetectionFilterRadius(camera_name, image_type, 200 * 100)  # in [cm]
    client.simAddDetectionFilterMeshName(camera_name, image_type, "ThirdPersonCharacter*")
    gaussian_weight = get_2d_gaussian_model()

    last_offset = 0
    find_obj = False
    # 逐步升高找寻目标
    objs = []
    while current_height < 100.0 and not find_obj:
        start_time = time.time()
        # 以一定角速度运动到某个高度
        client.rotateByYawRateAsync(360/round_time, round_time, vehicle_name)
        while time.time() - start_time < round_time:
            objs = client.simGetDetections(camera_name, image_type)
            if objs:
                client.cancelLastTask(vehicle_name)
                find_obj = True
                res = det_coord(objs)
                last_offset = res[2]
                print("have found")
                break
        current_height += add_height


    # 没有检测到目标
    if not find_obj:
        print("Not find target")
    # 检测到目标
    else:
        current_speed = 3
        print("Closing to target")

        while True:
            objs = client.simGetDetections(camera_name, image_type)
            # 暂时只考虑一个目标的情况
            if len(objs) == 0:
                continue
            res = det_coord(objs)
            # norm = res[2]
            # if norm <= camera_height/2. * (1./2.):
            #     continue
            direct_vec = (res[0], res[1])
            print("方向向量为：%s" % pprint.pformat(direct_vec))
            # yaw = math.atan2(v_dir_y, v_dir_x)
            # yaw = yaw / (2*pi) * 360

            # 获取当前无人机的位置信息
            drone_state = client.getMultirotorState(vehicle_name)
            angle_acc = drone_state.kinematics_estimated.angular_acceleration
            # 摆动太大，等待稳定
            if angle_acc.x_val > 0.5 or angle_acc.y_val > 0.5:
                print("摆动太大")
                # client.hoverAsync().join()
                continue

            # 位姿是四元数表示的
            orient = drone_state.kinematics_estimated.orientation
            # 四元数转化为欧拉角
            eular = airsim.to_eularian_angles(orient)
            # 旋转角度前面要乘以 -1
            new_vec = rotate_2vector(-1 * eular[2], direct_vec)
            # print("世界坐标的方向向量为：%s" % pprint.pformat(new_vec))
            # print("欧拉角为：")
            # print(eular[2]/2.0/pi * 360)

            # diff_offset = norm - last_offset
            # if diff_offset > 0:
            #     current_speed += 0.1
            # else:
            #     current_speed -= 0.1
            # last_offset = norm

            weighted_speed = current_speed * gaussian_weight[res[4], res[3]]
            print(weighted_speed)
            # 这里 vx,vy,vz都是无人机坐标NED下的位置
            client.moveToPositionAsync(new_vec[1]+drone_state.kinematics_estimated.position.x_val,
                                       new_vec[0]+drone_state.kinematics_estimated.position.x_val,
                                       -1 * current_height, weighted_speed,
                                       vehicle_name=vehicle_name)


    client.landAsync().join()
    client.reset()
    client.armDisarm(False)
    client.enableApiControl(False)


if __name__ == "__main__":
    main()
    # get_2d_gaussian_model()