from t_util import *

"""-------------------------------------------------drone task-------------------------------------------------------"""


class Task(threading.Thread):
    """
    任务的父类
    """

    def __init__(self, client: airsim.MultirotorClient, vehicle_name, init_pos):
        threading.Thread.__init__(self)
        self.client = client
        self.vehicle_name = vehicle_name
        self.init_pos = init_pos
        # 任务编号
        self.task_num = 0
        self.result = None

    def get_result(self):
        return self.result


class TaskSearch(Task):
    def __init__(self, client, vehicle_name, init_pos, min_height, max_height, area: Area, position=None):
        """
        搜索任务
        :param min_height: 最小搜索高度
        :param max_height: 最大搜索高度
        :param position: 要搜索的位置 ------------- A 转交追踪任务给 B 的时候用 -------------
        """
        Task.__init__(self, client, vehicle_name, init_pos)
        self.task_num = TaskState.SEARCH
        self.min_height = min_height
        self.max_height = max_height
        # class is airsim.Vector3r
        self.position = position
        self.area = area

    def run(self):
        # 数据准备
        drone_state = self.client.getMultirotorState(self.vehicle_name)
        if drone_state.landed_state == airsim.LandedState.Landed:
            self.client.takeoffAsync().join()
        if drone_state.kinematics_estimated.position.z_val > self.min_height:
            current_height = drone_state.kinematics_estimated.position.z_val
        else:
            current_height = self.min_height

        find_obj = False
        # 每次升高10m继续搜索
        add_height = 10.0
        # 每一圈用时10s
        round_time = 10.0
        self.client.simSetDetectionFilterRadius(camera_name, image_type, 200 * 100)  # in [cm]
        self.client.simAddDetectionFilterMeshName(camera_name, image_type, "ThirdPersonCharacter*")

        # 开始搜索
        # TODO 飞往目的地的时候无法捕捉目标，如有需要要改进
        if self.position is None:
            self.client.moveToZAsync(-1.0 * current_height, 5).join()
        else:
            final_pos = find_point_area(self.position, self.area)
            # 由其他无人机传来的位置转化为自己的相对位置
            final_pos = to_related_position(final_pos, self.init_pos)
            self.client.moveToPositionAsync(final_pos.x_val,
                                            final_pos.y_val,
                                            -1.0 * current_height,
                                            5, vehicle_name=self.vehicle_name).join()

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

    def __init__(self, client, vehicle_name, init_pos, track_height, area: Area, time_out=10):
        """
        执行跟踪任务
        :param track_height: 设定跟踪高度
        :return:
        """
        Task.__init__(self, client, vehicle_name, init_pos)
        self.task_num = TaskState.TRACK
        self.track_height = track_height
        # 10秒没有检测到目标则丢失目标
        self.time_out = time_out
        self.area = area
        self.in_area = True

    def run(self):
        objs = []
        speed_ctrl = PIDController(0.08, 0.005, 0.005, 8)
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

            lose_time = time.time()
            # TODO 判断是否在区域内
            self.in_area = in_area(to_world_position(drone_state.kinematics_estimated.position,
                                                  self.init_pos), self.area)
            if not self.in_area:
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
            if cur_speed < 1:
                current_height = self.track_height

            # 跟随
            if self.in_area:
                self.client.moveByVelocityZAsync(vx, vy, -1.0 * current_height, 2,
                                                 airsim.DrivetrainType.MaxDegreeOfFreedom,
                                                 yaw_mode,
                                                 vehicle_name=self.vehicle_name)
                self.result = TrackState.DOING
