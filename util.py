import inspect
import ctypes
import cv2
import numpy as np
import time
from PIDcontroller import PIDController
from types import *
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


def in_area(position: airsim.Vector3r, area: Area):
    """
    利用面积来判断一个点是不是在四边形内部
    """

    def triple_area(p1: Vector2r, p2: Vector2r, p3: Vector2r):
        """
        三角形计算公式，面积计算分解成三角形面积之和
        """
        return 0.5 * (p1.x_val * p2.y_val + p2.x_val * p3.y_val + p3.x_val * p1.y_val
                            - p1.x_val * p3.y_val - p2.x_val * p1.y_val - p3.x_val * p2.y_val)

    point = Vector2r(position.x_val, position.y_val)

    s1 = triple_area(point, area.p1, area.p2) + triple_area(point, area.p2, area.p3) \
         + triple_area(point, area.p3, area.p4) + triple_area(point, area.p1, area.p4)
    s2 = triple_area(area.p1, area.p2, area.p3) + triple_area(area.p1, area.p3, area.p4)
    # 面积相等则在内部
    if math.fabs(s1 - s2) < 1e-5:
        return True
    else:
        return False


def find_point_area(point: airsim.Vector3r, area: Area) -> Vector2r:
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


"""-------------------------------------------------drone task-------------------------------------------------------"""


class Task(threading.Thread):
    """
    任务的父类
    """

    def __init__(self, client: airsim.MultirotorClient, vehicle_name):
        threading.Thread.__init__(self)
        self.client = client
        self.vehicle_name = vehicle_name
        # 任务编号
        self.task_num = 0
        self.result = None

    def get_result(self):
        return self.result


class TaskSearch(Task):
    task_num = 4

    def __init__(self, min_height, max_height, area: Area, position=None):
        """
        搜索任务
        :param min_height: 最小搜索高度
        :param max_height: 最大搜索高度
        :param position: 要搜索的位置 ------------- A 转交追踪任务给 B 的时候用 -------------
        """
        Task.__init__(self)
        self.min_height = min_height
        self.max_height = max_height
        # class is airsim.Vector3r
        self.position = position
        self.area = area

    def run(self):
        drone_state = self.client.getMultirotorState(self.vehicle_name)
        if drone_state.landed_state == airsim.LandedState.Landed:
            self.client.takeoffAsync().join()

        current_height = self.min_height

        # TODO 飞往目的地的时候无法捕捉目标，如有需要要改进
        if self.position is None:
            self.client.moveToZAsync(-1.0 * current_height, 5).join()
        else:
            final_pos = find_point_area(self.position, self.area)
            self.client.moveToPositionAsync(final_pos.x_val,
                                            final_pos.y_val,
                                            -1.0 * current_height,
                                            5, vehicle_name=self.vehicle_name).join()

        # 每次升高10m继续搜索
        add_height = 10.0
        # 每一圈用时10s
        round_time = 10.0
        self.client.simSetDetectionFilterRadius(camera_name, image_type, 200 * 100)  # in [cm]
        self.client.simAddDetectionFilterMeshName(camera_name, image_type, "ThirdPersonCharacter*")

        find_obj = False
        # 逐步升高找寻目标
        objs = []
        while current_height < self.max_height and not find_obj:
            start_time = time.time()
            # 以一定角速度运动到某个高度
            self.client.rotateByYawRateAsync(360 / round_time, round_time, self.vehicle_name)
            while time.time() - start_time < round_time:
                objs = self.client.simGetDetections(camera_name, image_type)
                if objs:
                    self.client.cancelLastTask(self.vehicle_name)
                    find_obj = True
                    break

            current_height += add_height
            self.client.moveToZAsync(-1.0 * current_height, 5)
        # 没有检测到目标
        self.result = find_obj


class TaskTrack(Task):
    task_num = 3

    def __init__(self, track_height, area: Area, time_out=10):
        """
        执行跟踪任务
        :param track_height: 设定跟踪高度
        :return:
        """
        Task.__init__(self)
        self.track_height = track_height
        # 10秒没有检测到目标则丢失目标
        self.time_out = time_out
        self.area = area
        self.in_area = True

    def run(self):
        objs = []
        speed_ctrl = PIDController(0.001, 0.01, 0.01, 8)
        yaw_ctrl = PIDController(0.08, 0.01, 0.05, math.radians(8))

        drone_state = self.client.getMultirotorState(self.vehicle_name)
        current_height = drone_state.kinematics_estimated.position.z_val

        lose_time = time.time()
        while True:
            # 获取当前无人机的位置信息
            drone_state = self.client.getMultirotorState(self.vehicle_name)
            angle = airsim.to_eularian_angles(drone_state.kinematics_estimated.orientation)

            objs = self.client.simGetDetections(camera_name, image_type)
            # 暂时只考虑一个目标的情况
            if len(objs) == 0:
                self.client.cancelLastTask(self.vehicle_name)
                if not self.in_area:
                    # 目标出界
                    self.result = TrackState.OUTAREA
                    break
                else:
                    if time.time() - lose_time > self.time_out:
                        # 丢失目标
                        self.result = TrackState.LOSE
                        break
                    else:
                        continue

            # TODO 判断是否在区域内
            self.in_area = in_area(drone_state.kinematics_estimated.position, self.area)
            if not self.in_area:
                self.result = TrackState.OUTAREA
                self.client.cancelLastTask(self.vehicle_name)
                return

            res = det_coord(objs)
            cur_yaw = math.atan2(res[1], res[0])

            # 偏航增量
            yaw_add = yaw_ctrl.getOutput(pi / 2 - cur_yaw)
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
                current_height = self.track_height

            # 跟随
            if self.in_area:
                self.client.moveByVelocityZAsync(vx, vy, -current_height, 1,
                                                 airsim.DrivetrainType.MaxDegreeOfFreedom,
                                                 yaw_mode,
                                                 vehicle_name=self.vehicle_name)
                self.result = TrackState.DOING


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
    pass
    # get_2d_gaussian_model()
