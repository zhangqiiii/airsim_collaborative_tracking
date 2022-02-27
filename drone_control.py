import time

import airsim

from t_task import *


class DroneControl(threading.Thread):
    """
    单个无人机的控制
    """

    def __init__(self, vehicle_name, client: airsim.MultirotorClient, area: Area, init_pos):
        threading.Thread.__init__(self)
        self.vehicle_name = vehicle_name
        self.client = client
        self.message = None
        self.area = area
        self.init_pos = tuple_to_vector3r(init_pos)
        self.task_state = TaskState.LAND
        self.current_task = None
        self.last_message = None
        # 内部主动结束无人机活动标志
        self.stop_flag = False
        self.drone_state = None
        """为了防止冲突，这里单独新建一个 client, self.client 用于任务执行，任务执行是串行的"""
        self.tmp_client = None

    def send_message(self, message):
        self.message = message

    def run(self):
        camera_pose = airsim.Pose(position_val=airsim.Vector3r(0.5, 0.0, 0.1),
                                  orientation_val=airsim.to_quaternion(-pi / 4., 0, 0))
        self.client.simSetCameraPose(camera_name, camera_pose, self.vehicle_name)
        self.client.armDisarm(True)

        # 循环处理事件
        self.tmp_client = airsim.MultirotorClient()
        while not self.stop_flag:
            # 不断监听消息并处理
            self.listen_message()
            # 判断任务的执行情况
            if self.current_task is not None:
                self.task_exec_state()
            # time.sleep(0.1)

    def stop(self):
        self.stop_flag = True
        self.client.goHomeAsync(vehicle_name=self.vehicle_name)

    def switch_current_task(self, task: Task):
        """
        切换任务
        """
        # 如果上个任务未结束，直接杀掉
        if self.current_task is not None:
            if self.current_task.is_alive():
                try:
                    stop_thread(self.current_task)
                except ValueError:
                    print("stop thread Error, that's ok, continue run")
        self.current_task = task
        self.current_task.start()
        self.task_state = task.task_num

    def listen_message(self):
        if self.last_message != self.message:
            self.last_message = self.message
            # 解析消息
            if self.message.m_type == MType.GET_STATE:
                self.drone_state = self.tmp_client.getMultirotorState(self.vehicle_name)
                position = self.drone_state.kinematics_estimated.position
                state = OppoState(position, self.task_state)
                self.message.set_res_data(state)
                self.message.set_status(1)
            elif self.message.m_type == MType.TRACK_TARGET:
                # 开始新的任务
                # TODO area 参数还没给
                current_task = TaskSearch(self.client, self.vehicle_name, self.init_pos, self.area,
                                          position=self.message.req_data)
                self.switch_current_task(current_task)
            elif self.message.m_type == MType.SEARCH_TARGET:
                current_task = TaskSearch(self.client, self.vehicle_name, self.init_pos, self.area)
                self.switch_current_task(current_task)
            else:
                pass

    def task_exec_state(self):
        # 搜索完毕
        if self.task_state == TaskState.SEARCH:
            # 搜索到目标，进入跟踪状态
            # print("搜索结果", self.current_task.result)
            if self.current_task.result:
                # TODO area 参数还没给
                current_task = TaskTrack(self.client, self.vehicle_name, self.init_pos, g_track_height, self.area)
                current_task.set_init_height(self.current_task.data)
                self.switch_current_task(current_task)
            elif self.current_task.result is None:
                pass
            # 没有搜索到目标
            else:
                # 任务已经结束才能判定没有搜索到目标
                if not self.current_task.is_alive():
                    self.task_state = TaskState.HOVER

        # 当前正在跟踪
        elif self.task_state == TaskState.TRACK:
            if self.current_task.result == TrackState.DOING:
                # print("DOING")
                pass

            elif self.current_task.result == TrackState.LOSE:
                print("LOSE 继续搜索")
                current_task = TaskSearch(self.client, self.vehicle_name, self.init_pos, self.area)
                self.switch_current_task(current_task)

            elif self.current_task.result == TrackState.OUTAREA:
                if not self.current_task.is_alive():
                    # print("real pose")
                    # print(self.client.simGetObjectPose("ThirdPersonCharacter_2"))
                    # print("OUTAREA")
                    data = self.current_task.data
                    print("eval pose")
                    print(data)
                    # task = Task(self.client, self.vehicle_name, self.init_pos)
                    # self.switch_current_task(task)
                    current_task = TaskGoHome(self.client, self.vehicle_name, self.init_pos)
                    self.switch_current_task(current_task)
                    # 广播
                    self.drone_state = self.tmp_client.getMultirotorState(self.vehicle_name)
                    track_task = Message(MType.TRACK_TARGET, threading.current_thread())
                    for drone_ in drone_list:
                        if drone_ != threading.current_thread():
                            # data 已经是世界坐标，不需要再进行转换
                            track_task.set_req_data(data)
                            drone_.send_message(track_task)

        else:
            pass


if __name__ == "__main__":
    position_PID_gains = airsim.PositionControllerGains(airsim.PIDGains(0.25, 0.1, 0.1),
                                                        airsim.PIDGains(0.25, 0.1, 0.1),
                                                        airsim.PIDGains(0.25, 0.1, 0.1))
    angle_PID_gains = airsim.AngleRateControllerGains(airsim.PIDGains(0.25, 0.01, -0.01),
                                                      airsim.PIDGains(0.25, 0.01, -0.01),
                                                      airsim.PIDGains(0.25, 0.01, -0.01))
    velocityPID = airsim.VelocityControllerGains(airsim.PIDGains(0.1, 0.01, 0.1),
                                                 airsim.PIDGains(0.1, 0.01, 0.1),
                                                 airsim.PIDGains(2.0, 2.0, 0))
    vehicle_name_list = ["Drone1", "Drone2"]
    client_list = [None, None]
    drone_list = [None, None]

    for i in range(2):
        client_list[i] = airsim.MultirotorClient()
        client_list[i].confirmConnection()
        # client_list[i].setAngleRateControllerGains(angle_PID_gains, vehicle_name_list[i])
        # client_list[i].setVelocityControllerGains(velocityPID, vehicle_name_list[i])
        client_list[i].enableApiControl(True, vehicle_name_list[i])
        drone_list[i] = DroneControl(vehicle_name_list[i], client_list[i],
                                     list_to_area(all_area[i]), init_position[i])
        drone_list[i].start()
        print(vehicle_name_list[i], " start")
    begin_task = Message(3, drone_list[0])
    drone_list[0].send_message(begin_task)
