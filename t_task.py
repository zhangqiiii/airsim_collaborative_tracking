from PIDcontroller import PIDController
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
        self.task_num = -1
        self.result = None
        # 添加附加数据
        self.data = None

    def get_result(self):
        return self.result

    def run(self):
        pass


class TaskSearch(Task):
    def __init__(self, client, vehicle_name, init_pos, area: Area,
                 min_height=g_search_min, max_height=g_search_max,
                 target="ThirdPersonCharacter*", position=None):
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
        self.target = target

    def run(self):
        print(self.vehicle_name, ": searching")
        # 数据准备
        drone_state = self.client.getMultirotorState(self.vehicle_name)
        if drone_state.landed_state == airsim.LandedState.Landed:
            self.client.takeoffAsync(vehicle_name=self.vehicle_name).join()
        if -drone_state.kinematics_estimated.position.z_val > self.min_height:
            current_height = -drone_state.kinematics_estimated.position.z_val
        else:
            current_height = self.min_height

        find_obj = False
        # 每次升高10m继续搜索
        add_height = g_add_height
        # 每一圈用时10s
        round_time = 10.0
        self.client.simSetDetectionFilterRadius(camera_name, image_type, 200 * 100, self.vehicle_name)  # in [cm]
        self.client.simAddDetectionFilterMeshName(camera_name, image_type, self.target, self.vehicle_name)

        # 开始搜索
        # TODO 飞往目的地的时候无法捕捉目标，如有需要要改进
        if self.position is None:
            self.client.moveToZAsync(-1.0 * current_height, 5, vehicle_name=self.vehicle_name).join()
        else:
            final_pos = find_point_area(self.position, self.area)
            # 由其他无人机传来的位置转化为自己的相对位置
            final_pos = to_related_position(final_pos, self.init_pos)
            # self.client.moveToPositionAsync(final_pos.x_val,
            #                                 final_pos.y_val,
            #                                 -1.0 * current_height,
            #                                 10, vehicle_name=self.vehicle_name).join()
            moveToPositionWithLidar(self.client, final_pos.x_val, final_pos.y_val,
                                    -1.0 * current_height, 8, self.vehicle_name)
        # 逐步升高找寻目标
        while True:
            drone_state = self.client.getMultirotorState(self.vehicle_name)
            start_yaw = airsim.to_eularian_angles(drone_state.kinematics_estimated.orientation)[2]
            start_time = time.time()
            # 以一定角速度运动到某个高度
            self.client.rotateByYawRateAsync(360 / round_time, round_time, self.vehicle_name)
            while True:
                objs = self.client.simGetDetections(camera_name, image_type, self.vehicle_name)
                drone_state = self.client.getMultirotorState(self.vehicle_name)
                delt_yaw = airsim.to_eularian_angles(drone_state.kinematics_estimated.orientation)[2] - start_yaw
                # 认为转了一圈了
                if delt_yaw < -math.radians(3):
                    break

                if len(objs) != 0:
                    self.client.cancelLastTask(self.vehicle_name)
                    self.data = current_height
                    find_obj = True
                    self.result = find_obj
                    break

            current_height += add_height

            if current_height >= self.max_height or find_obj:
                print("current_height", current_height)
                break

            self.client.moveToZAsync(-1.0 * current_height, 5, vehicle_name=self.vehicle_name).join()
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
        self.init_height = None

    def set_init_height(self, height):
        self.init_height = height

    def run(self):
        print(self.vehicle_name, ": tracking")
        speed_ctrl = PIDController(*g_speed_PID)
        yaw_ctrl = PIDController(*g_yaw_PID)

        lidar_ctrl = droneLidar(self.client, self.vehicle_name)

        current_height = self.init_height

        lose_time = time.time()
        start_time = lose_time
        while True:
            # 获取当前无人机的位置信息
            drone_state = self.client.getMultirotorState(self.vehicle_name)
            angle = airsim.to_eularian_angles(drone_state.kinematics_estimated.orientation)
            current_position = drone_state.kinematics_estimated.position
            objs = self.client.simGetDetections(camera_name, image_type, self.vehicle_name)

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

            det_res = det_coord(objs, -current_position.z_val)
            cur_yaw = math.atan2(det_res[1], det_res[0])

            # TODO 判断是否在区域内
            self.in_area = in_area(to_world_position(current_position, self.init_pos), self.area)
            if not self.in_area:
                self.result = TrackState.OUTAREA
                self.client.cancelLastTask(self.vehicle_name)
                data_tmp = eval_target_position(self.client, self.vehicle_name, Vector2r(det_res[0], det_res[1]))
                self.data = to_world_position(data_tmp, self.init_pos)
                return

            # 偏航增量
            yaw_add = yaw_ctrl.getOutput(pi / 2 - cur_yaw)
            cur_speed = speed_ctrl.getOutput(det_res[2])

            vy = cur_speed * math.sin(cur_yaw)
            vx = cur_speed * math.cos(cur_yaw)
            vy, vx = rotate_2vector(-angle[2], (vx, vy))

            # 判断是否需要偏航
            if 0 < cur_yaw < pi:
                yaw_mode = airsim.YawMode(False, math.degrees(angle[2] + yaw_add))
            else:
                yaw_mode = airsim.YawMode(True, 0)

            # 避障检测
            # sensing_time = time.time()
            lidar_res = lidar_ctrl.sensing()
            if lidar_res != 0:
                print(lidar_res)
            if lidar_res == voidObstacleAction.DANGER:
                current_height = -current_position.z_val + g_danger_add_height
                self.client.moveToZAsync(-current_height,
                                         g_position_speed, vehicle_name=self.vehicle_name)
            else:
                if lidar_res == voidObstacleAction.WARN:
                    current_height = -current_position.z_val + g_danger_add_height
                elif lidar_res == voidObstacleAction.KEEP:
                    current_height = current_position.z_val
                if math.fabs(angle[0]) < g_cancel_det_angle and math.fabs(angle[1]) < g_cancel_det_angle:
                    self.client.moveByVelocityZAsync(vx, vy, -current_height, 4,
                                                     airsim.DrivetrainType.MaxDegreeOfFreedom,
                                                     yaw_mode,
                                                     vehicle_name=self.vehicle_name)
                # 机体倾斜，不执行追踪
                else:
                    pass
            self.result = TrackState.DOING

            # 目标在中间区域后适当降低高度
            if cur_speed < g_descent_speed and time.time() - start_time > 1 and \
                    det_res[2] < camera_width * g_descent_range:
                current_height = self.track_height


class TaskGoHome(Task):
    def __init__(self, client, vehicle_name, init_pos):
        Task.__init__(self, client, vehicle_name, init_pos)
        self.task_num = TaskState.RETURN
        self.target = Vector3r(0, 0, -0.5)

    def run(self):
        print(self.vehicle_name, ": going home")
        moveToPositionWithLidar(self.client, self.target.x_val, self.target.y_val, self.target.z_val - 10.0,
                                g_position_speed, self.vehicle_name)
        self.client.moveToZAsync(self.target.z_val, g_position_speed,
                                 vehicle_name=self.vehicle_name).join()
        print("landing")
        self.client.landAsync(vehicle_name=self.vehicle_name).join()
        print("landed")
        self.result = True
