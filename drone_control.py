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

    def send_message(self, message):
        self.message = message

    def run(self):
        camera_pose = airsim.Pose(position_val=airsim.Vector3r(0.5, 0.0, 0.1),
                                  orientation_val=airsim.to_quaternion(-pi / 4., 0, 0))
        self.client.simSetCameraPose(camera_name, camera_pose, self.vehicle_name)
        self.client.armDisarm(True)

        # 循环处理事件
        while not self.stop_flag:
            self.drone_state = self.client.getMultirotorState(self.vehicle_name)
            # 不断监听消息并处理
            self.listen_message()
            # 判断任务的执行情况
            if self.current_task:
                self.task_exec_state()

    def stop(self):
        self.stop_flag = True
        self.client.goHomeAsync(vehicle_name=self.vehicle_name)

    def listen_message(self):
        if self.last_message != self.message:
            self.last_message = self.message
            # 解析消息
            if self.message.m_type == 1:
                position = self.drone_state.kinematics_estimated.position
                state = OppoState(position, self.task_state)
                self.message.set_res_data(state)
                self.message.set_status(1)
            elif self.message.m_type == 2:
                # 终止当前任务
                if self.current_task is not None:
                    stop_thread(self.current_task)
                # 开始新的任务
                # TODO area 参数还没给
                current_task = TaskSearch(self.client, self.vehicle_name, self.init_pos, 20, 50, self.area)
                self.current_task = current_task
                self.task_state = TaskState.SEARCH
                current_task.start()
                self.task_state = TaskState.SEARCH
            elif self.message.m_type == 3:
                current_task = TaskSearch(self.client, self.vehicle_name, self.init_pos, 20, 50, self.area)
                current_task.start()
                self.current_task = current_task
                self.task_state = TaskState.SEARCH
            else:
                pass

    def task_exec_state(self):
        # 搜索完毕
        if self.task_state == TaskState.SEARCH:
            # 搜索到目标，进入跟踪状态
            if self.current_task.result:
                # TODO area 参数还没给
                current_task = TaskTrack(self.client, self.vehicle_name, self.init_pos, 15, self.area)
                self.current_task = current_task
                current_task.start()
                self.task_state = TaskState.TRACK
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
                current_task = TaskSearch(self.client, self.vehicle_name, self.init_pos, 20, 50, self.area)
                current_task.start()
                self.current_task = current_task
                self.task_state = TaskState.SEARCH
            elif self.current_task.result == TrackState.OUTAREA:
                print("OUTAREA")
                # 广播
                track_task = Message(2, threading.current_thread())
                for drone_ in drone_list:
                    if drone_list != threading.current_thread():
                        # 将自身的位置转化为世界坐标
                        track_task.req_data(to_world_position(self.drone_state.kinematics_estimated.position, self.init_pos))
                        drone_.send_message(track_task)

        else:
            pass


if __name__ == "__main__":
    position_PID_gains = airsim.PositionControllerGains(airsim.PIDGains(0.25, 0.1, 0.1),
                                                        airsim.PIDGains(0.25, 0.1, 0.1),
                                                        airsim.PIDGains(0.25, 0.1, 0.1))
    vehicle_name_list = ["Drone1", "Drone2"]
    client_list = [None, None]
    drone_list = [None, None]
    for i in range(2):
        client_list[i] = airsim.MultirotorClient()
        client_list[i].confirmConnection()
        # client_list[i].setPositionControllerGains(position_PID_gains, vehicle_name1)
        client_list[i].enableApiControl(True, vehicle_name_list[i])
        drone_list[i] = DroneControl(vehicle_name_list[i], client_list[i],
                                     list_to_area(all_area[0]), init_position[0])
        drone_list[i].start()
        print(vehicle_name_list[i], " start")
    begin_task = Message(3, drone_list[0])
    drone_list[0].send_message(begin_task)
