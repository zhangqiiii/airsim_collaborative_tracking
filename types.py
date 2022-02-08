import threading

import airsim


class MType:
    """
    定义消息类型
        1 表示获取对方状态(任务状态和位置信息)
        2 表示通知目标位置，让对应区域无人机进行跟踪任务
        ...
        其他数字预留
    """
    GET_STATE = 1
    TRACK_TARGET = 2


class Message:
    """
    消息类，用于封装线程间通信信息
    """
    def __init__(self, m_type: int, target: threading.Thread):
        """
        :param m_type: 消息类型，从 1 开始
        :param target: 消息的发送目标
        """
        self.m_type = m_type
        self.source = threading.current_thread()
        self.target = target
        # 附加数据，可选项，没有固定类型，一般情况下都是对象的引用
        self.req_data = None
        self.status = -1
        self.res_data = None

    def set_req_data(self, req_data):
        """
        数据需要单独赋值
        """
        self.req_data = req_data

    def set_status(self, m_status=1):
        """
        1 表示成功处理或处理中；0 表示收到但是无法执行；-1表示还未收到
        """
        self.status = m_status

    def set_res_data(self, res_data):
        self.res_data = res_data


class TaskState:
    """
    无人机的状态枚举，写成类方便根据单词意思赋值
        0 停机
        1 巡逻
        2 悬停（没有找到目标或者丢失目标）
        3 跟踪中
        4 搜索
        5 返航（返航持续时间会比较长，可以单独作为一个状态）
    """
    LAND = 0
    PATROL = 1
    HOVER = 2
    TRACK = 3
    SEARCH = 4
    RETURN = 5


class OppoState:
    def __init__(self, position: airsim.Vector3r, task: TaskState):
        self.position = position
        self.task = task


class TrackState:
    """
    追踪状态（由于追踪可能会丢失目标）
    丢失、正在执行、目标越界
    """
    LOSE = 0
    DOING = 1
    OUTAREA = 2


class Area:
    """
    每个无人机的值守区域，简单区域，凸四边形；且四个点为顺时针排列
    """
    def __init__(self, p1: airsim.Vector2r, p2, p3, p4):
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.p4 = p4

