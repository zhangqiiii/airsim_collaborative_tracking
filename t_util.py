import inspect
import ctypes
from airsim import Vector2r
import cv2
import numpy as np
import time
from typing import List
from t_types import *
# 导入全局参数
from param_setting import *

"""---------------------------------------------utilize function-----------------------------------------------------"""


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


def det_coord(objs, current_height):
    """
    返回目标的检测结果
    :param objs: simGetDetections的检测结果
    :param current_height: 当前的高度
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


# def get_2d_gaussian_model():
#     """
#     返回一个二维高斯核，用来调节在不同视野位置下无人机的速度
#     :return: 返回值是速度的权重图，一次计算，重复使用
#     """
#     upsample = 12
#     h_sigma = 6
#     w_sigma = h_sigma * camera_width / camera_height
#     row = cv2.getGaussianKernel(int(camera_height / upsample), h_sigma)
#     col = cv2.getGaussianKernel(int(camera_width / upsample), int(w_sigma)).T
#     model = row * col
#     max_value = np.max(model)
#     model = model / max_value
#     model = cv2.resize(model, (camera_width, camera_height))
#     # cv2.imshow("e", model)
#     # cv2.waitKey()
#     return 1 - model


def in_area(position: Vector3r, area: Area, error=1e-3):
    """
    利用面积来判断一个点是不是在四边形内部
    """

    def triple_area(p1: Vector2r, p2: Vector2r, p3: Vector2r):
        """
        三角形计算公式，面积计算分解成三角形面积之和
        """
        return 0.5 * math.fabs((p1.x_val * p2.y_val + p2.x_val * p3.y_val + p3.x_val * p1.y_val
                                - p1.x_val * p3.y_val - p2.x_val * p1.y_val - p3.x_val * p2.y_val))

    point = Vector2r(position.x_val, position.y_val)

    s1 = triple_area(point, area.p1, area.p2) + triple_area(point, area.p2, area.p3) \
         + triple_area(point, area.p3, area.p4) + triple_area(point, area.p1, area.p4)
    s2 = triple_area(area.p1, area.p2, area.p3) + triple_area(area.p1, area.p3, area.p4)
    # 面积相等则在内部
    if math.fabs(s1 - s2) < error:
        return True
    else:
        # print(math.fabs(s1 - s2), " position", position.x_val, " ", position.y_val)
        return False


def find_point_area(point: Vector3r, area: Area, bias=2) -> Vector2r:
    """
    寻找给定点里给定区域最近的点
    bias 为偏置量，保证找到的点在自己区域内，避免一些问题
    :return: 
    """

    if in_area(point, area):
        return Vector2r(point.x_val, point.y_val)
    else:
        """
        寻找离地最近的顶点，判断两个邻点和给定点之间的向量角度，具体思路看代码
        """
        point_ = np.array([point.x_val, point.y_val])
        vertex_list = [area.p1, area.p2, area.p3, area.p4]
        point_num = len(vertex_list)
        min_ind = 0
        min_norm = float("inf")
        for i, p_ in enumerate(vertex_list):
            vertex_list[i] = np.array([p_.x_val, p_.y_val])
        for i in range(point_num):
            j = (i + 1) % point_num
            distance = np.cross(vertex_list[i] - point_, vertex_list[j] - point_) / \
                       np.linalg.norm(vertex_list[i] - vertex_list[j])
            if distance < 0:
                distance = min(np.linalg.norm(vertex_list[i] - point_), np.linalg.norm(vertex_list[j] - point_))
            if distance < min_norm:
                min_norm = distance
                min_ind = i

        # 距离最近的边的两个端点
        p_1 = vertex_list[min_ind]
        p_2 = vertex_list[(min_ind + 1) % point_num]

        # 求三个向量
        m_vec = p_1 - p_2
        p1_vec = p_1 - point_
        p2_vec = p_2 - point_

        # 求两个夹角
        angle1 = m_vec.dot(p1_vec) / (np.linalg.norm(m_vec) * np.linalg.norm(p1_vec))
        angle2 = m_vec.dot(-1.0 * p2_vec) / (np.linalg.norm(m_vec) * np.linalg.norm(p2_vec))

        if angle1 <= 0 and angle2 > 0:
            return Vector2r(p_1[0], p_1[1])
        elif angle2 <= 0 and angle1 > 0:
            return Vector2r(p_2[0], p_2[1])
        else:
            project_len = m_vec.dot(p1_vec) / np.linalg.norm(m_vec)
            vec_len = np.linalg.norm(m_vec)
            ratio = project_len / vec_len
            new_point = p_1 - ratio * m_vec
            bias_vec = (new_point - point_) / np.linalg.norm(new_point - point_) * bias
            new_point = new_point + bias_vec
            return Vector2r(new_point[0], new_point[1])


def list_to_area(point_list: List[tuple]):
    """
    将 param_setting.py 中易书写的格式转化为 Area 类
    """
    return Area(Vector2r(point_list[0][0], point_list[0][1]), Vector2r(point_list[1][0], point_list[1][1]),
                Vector2r(point_list[2][0], point_list[2][1]), Vector2r(point_list[3][0], point_list[3][1]))


def tuple_to_vector3r(pos_tuple):
    return Vector3r(pos_tuple[0], pos_tuple[1], pos_tuple[2])


def to_world_position(position: [Vector3r, Vector2r], init_pos: Vector3r):
    position_res = Vector3r()
    position_res.x_val = position.x_val + init_pos.x_val
    position_res.y_val = position.y_val + init_pos.y_val
    if isinstance(position, Vector3r):
        position_res.z_val = position.z_val + init_pos.z_val
    return position_res


def to_related_position(position: [Vector3r, Vector2r], init_pos: Vector3r):
    position_res = Vector3r()
    position_res.x_val = position.x_val - init_pos.x_val
    position_res.y_val = position.y_val - init_pos.y_val
    if isinstance(position, Vector3r):
        position_res.z_val = position.z_val - init_pos.z_val
    return position_res


def eval_target_position(client: airsim.MultirotorClient, vehicle_name, target_position: Vector2r):
    drone_state = client.getMultirotorState(vehicle_name)
    drone_position = drone_state.kinematics_estimated.position
    orientation_euler = airsim.to_eularian_angles(drone_state.kinematics_estimated.orientation)
    # drone_pitch = orientation_euler[0]
    drone_pitch = 0.0
    camera_info = client.simGetCameraInfo(camera_name, vehicle_name)
    # UE4 中的 FOV 其实是 HFOV（水平视场角）
    fov_x = camera_info.fov.real
    fov_y = fov_x * camera_height / (camera_width * 1.0)
    camera_pitch = -1.0 * airsim.to_eularian_angles(camera_info.pose.orientation)[0]
    height_line = math.fabs(drone_position.z_val)
    real_pitch = camera_pitch - drone_pitch

    center_line = height_line / math.sin(real_pitch)
    center_y = height_line / math.tan(real_pitch)
    oppo_line_y = center_line * math.tan(math.radians(fov_y) / 2.0)

    # 相对无人机垂点 y 轴的地面距离（机身坐标系）
    if target_position.y_val > 0:
        y = (target_position.y_val / camera_height * oppo_line_y) / math.sin(real_pitch) + center_y
    else:
        y = center_y - (target_position.y_val / camera_height * oppo_line_y) / math.cos(real_pitch)

    oppo_line_x = center_line * math.tan(math.radians(fov_x) / 2.0)
    # 相对无人机垂点 x 轴的地面距离（机身坐标系）
    x = target_position.x_val / camera_width * oppo_line_x

    # NED 坐标系下该点相对无人机垂点的地面坐标
    y, x = rotate_2vector(-orientation_euler[2], (x, y))

    return Vector2r(x + drone_position.x_val, y + drone_position.y_val)


def position_dist(pos1: [Vector3r, Vector2r], pos2: [Vector3r, Vector2r]) -> float:
    """
    计算两个 Vector3r 或者 Vector2r的距离
    """
    assert type(pos2) == type(pos1)
    if isinstance(pos1, Vector2r):
        a = np.array((pos1.x_val, pos1.y_val))
        b = np.array((pos2.x_val, pos2.y_val))
    else:
        a = np.array((pos1.x_val, pos1.y_val, pos1.z_val))
        b = np.array((pos2.x_val, pos2.y_val, pos2.z_val))
    return np.linalg.norm(a - b)


def moveToPositionWithLidar(client: airsim.MultirotorClient, x, y, z, velocity, vehicle_name, timeout=10):
    """
    带有避障功能的 move to position
    timeout 表示位置稳定后超时则表示已完成
    """
    lidar_ctrl = droneLidar(client, vehicle_name)
    # 目标地点
    target = Vector3r(x, y, z)
    his_dist = historyData(5)

    while True:
        drone_state = client.getMultirotorState(vehicle_name)
        drone_position = drone_state.kinematics_estimated.position
        line_dist = position_dist(drone_position, target)
        his_dist.update(line_dist)
        # print(line_dist)
        stable_time = None
        stable = his_dist.is_stable(thr=0.5)
        # 认为到达目的地的条件1
        if stable:
            if stable_time is None:
                stable_time = time.time()
            else:
                if time.time() - stable_time > timeout:
                    break
        else:
            stable_time = None
        # 认为到达目的地的条件2
        if line_dist <= 5 and stable:
            break

        res = lidar_ctrl.sensing()
        # print(res)
        if res == voidObstacleAction.WARN:
            client.moveToPositionAsync(x, y, z - line_dist, velocity, vehicle_name=vehicle_name)
        elif res == voidObstacleAction.DANGER:
            client.moveToZAsync(drone_state.kinematics_estimated.position.z_val - g_danger_add_height,
                                velocity, vehicle_name=vehicle_name)
        elif res == voidObstacleAction.KEEP:
            client.moveToPositionAsync(x, y, drone_state.kinematics_estimated.position.z_val,
                                       velocity, vehicle_name=vehicle_name)
        else:
            client.moveToPositionAsync(x, y, z, velocity, vehicle_name=vehicle_name)
    client.hoverAsync(vehicle_name=vehicle_name)
    print("arrive")


"""----------------------------------------------------task schedule-------------------------------------------------"""


def _async_raise(tid, exctype):
    """raises the exception, performs cleanup if needed"""
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        # """if it returns a number greater than one, you're in trouble,
        # and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")


def stop_thread(thread: threading.Thread):
    """
    外部强行结束线程
    """
    _async_raise(thread.ident, SystemExit)


"""---------------------------------------------------avoid obstacle-------------------------------------------------"""


class lidarTmpData:
    """
    存储激光雷达的之前的若干（三组）数据，判断障碍物
    """

    def __init__(self, name, bottom=False):
        self.name = name
        self.bottom = bottom
        # self.pre = None
        self.last = float("inf")
        self.now = float("inf")

    def update(self, new_data):
        # self.pre = self.last
        self.last = self.now
        self.now = new_data if new_data is not None else float("inf")
        # print(self.name, self.last, self.now)

    def is_obstacle(self):
        """
        判断是否正在靠近障碍物
        :return: 2 表示正在靠近，采取在移动的过程中升高高度的策略；
                 3 表示十分靠近了，必须停下当前的移动指令，进行升高操作
                 1 表示不要下降
                 0 表示没有靠近障碍或者没有障碍
        """
        if self.bottom:
            if self.now < g_danger_dist:
                return voidObstacleAction.KEEP
            else:
                return voidObstacleAction.NO
        if self.last == float("inf") and self.now == float("inf"):
            return voidObstacleAction.NO
        if self.now <= self.last <= g_warn_dist:
            if self.now > g_danger_dist:
                return voidObstacleAction.WARN
            else:
                return voidObstacleAction.DANGER
        else:
            return voidObstacleAction.NO


class droneLidar:
    """
    激光雷达控制器，用来检测障碍
    """

    def __init__(self, client, vehicle_name):
        self.client = client
        self.vehicle_name = vehicle_name
        # 一个检测四周，一个检测下面
        self.name = ["F", "B"]
        self.lidar = []
        for i in self.name:
            self.lidar.append(lidarTmpData(i, True if i == "B" else False))

    def sensing(self):
        res = 0
        for lidar in self.lidar:
            dist_list = []
            lidar_data = self.client.getLidarData("Lidar" + lidar.name, self.vehicle_name)
            tmp = lidar_data.point_cloud
            if len(tmp) > 3:
                for i in range(0, len(tmp), 3):
                    dist_list.append(np.linalg.norm((tmp[i], tmp[i + 1], tmp[i + 2])))

            if len(dist_list) == 0:
                lidar.update(None)
            else:
                lidar.update(min(dist_list))
            # 选择有风险事件
            res = max(res, lidar.is_obstacle())
        return res


if __name__ == "__main__":
    tmp_area = [(0, 0), (0, 20), (20, 20), (20, 0)]
    area_ = list_to_area(tmp_area)
    pos_ = Vector3r(12, 23, -11)
    # p = in_area(pos_, area_)
    p = find_point_area(pos_, area_)
    print("p", p)
