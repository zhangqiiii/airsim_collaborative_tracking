import inspect
import ctypes
import cv2
import numpy as np
import time

from typing import List

from PIDcontroller import PIDController
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


from airsim import Vector2r


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
        print(math.fabs(s1 - s2))
        return False


def find_point_area(point: Vector3r, area: Area) -> Vector2r:
    """
    寻找给定点里给定区域最近的点
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
        area_ = enumerate(vertex_list)
        min_ind = 0
        min_norm = float("inf")
        for i, p in area_:
            vertex_list[i] = np.array([p.x_val, p.y_val])
            norm = np.linalg.norm(vertex_list[i] - point_)
            if norm < min_norm:
                min_ind = i
                min_norm = norm
        # 相邻的两个点
        p_1 = (min_norm - 1) % len(vertex_list)
        p_2 = (min_norm + 1) % len(vertex_list)

        # 求三个向量
        m_vec = point - vertex_list[min_ind]
        p1_vec = p_1 - vertex_list[min_ind]
        p2_vec = p_2 - vertex_list[min_ind]

        # 求两个夹角
        angle1 = m_vec.dot(p1_vec) / (np.linalg.norm(m_vec) * np.linalg.norm(p1_vec))
        angle2 = m_vec.dot(p2_vec) / (np.linalg.norm(m_vec) * np.linalg.norm(p2_vec))

        p_vec = None
        if angle1 <= 0 and angle2 <= 0:
            return Vector2r(point.x_val, point.y_val)
        elif angle1 <= 0 and angle2 > 0:
            p_vec = p2_vec
        elif angle2 <= 0 and angle1 > 0:
            p_vec = p1_vec
        else:
            print("Error")
        if p_vec:
            project_len = m_vec.dot(p_vec) / np.linalg.norm(m_vec)
            vec_len = np.linalg.norm(p_vec)
            ratio = project_len / vec_len
            new_vec = m_vec + ratio * (p_vec - m_vec)
            return Vector2r(new_vec[0], new_vec[1])


def list_to_area(point_list: List[tuple]):
    """
    将 param_setting.py 中易书写的格式转化为 Area 类
    """
    return Area(Vector2r(point_list[0][0], point_list[0][1]), Vector2r(point_list[1][0], point_list[1][1]),
                Vector2r(point_list[2][0], point_list[2][1]), Vector2r(point_list[3][0], point_list[3][1]))


def tuple_to_vector3r(pos_tuple):
    return Vector3r(pos_tuple[0], pos_tuple[1], pos_tuple[2])


def to_world_position(position: [Vector3r, Vector2r], init_pos: Vector3r):
    position.x_val += init_pos.x_val
    position.y_val += init_pos.y_val
    if isinstance(position, Vector3r):
        position.z_val += init_pos.z_val
    return position


def to_related_position(position: [Vector3r, Vector2r], init_pos: Vector3r):
    position.x_val -= init_pos.x_val
    position.y_val -= init_pos.y_val
    if isinstance(position, Vector3r):
        position.z_val -= init_pos.z_val
    return position


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


if __name__ == "__main__":
    area_ = list_to_area(all_area[0])
    pos_ = Vector3r(2, 3, -11)
    p = in_area(pos_, area_)
    print(p)

