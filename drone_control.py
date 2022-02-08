from util import *
from types import *


class DroneControl(threading.Thread):
    """
    单个无人机的控制
    """

    def __init__(self, vehicle_name, client: airsim.MultirotorClient, area):
        threading.Thread.__init__(self)
        self.vehicle_name = vehicle_name
        self.client = client
        self.message = None
        self.area = area
        self.task_state = TaskState.LAND
        self.current_task = None
        self.last_message = None
        # 内部主动结束无人机活动标志
        self.stop_flag = False

    def send_message(self, message):
        self.message = message

    def run(self):
        camera_pose = airsim.Pose(position_val=airsim.Vector3r(0.5, 0.0, 0.1),
                                  orientation_val=airsim.to_quaternion(-pi / 4., 0, 0))
        self.client.simSetCameraPose(camera_name, camera_pose, self.vehicle_name)
        self.client.armDisarm(True)

        # 循环处理事件
        while not self.stop_flag:
            # 不断监听消息并处理
            self.listen_message()
            # 判断任务的执行情况
            if not self.current_task.is_alive():
                self.task_exec_state()

    def listen_message(self):
        if self.last_message != self.message:
            self.last_message = self.message
            # 解析消息
            if self.message.m_type == 1:
                position = self.client.getMultirotorState(self.vehicle_name) \
                    .kinematics_estimated.position
                state = OppoState(position, self.task_state)
                self.message.set_res_data(state)
                self.message.set_status(1)
            elif self.message.m_type == 2:
                # 终止当前任务
                if self.current_task is not None:
                    stop_thread(self.current_task)
                # 开始新的任务
                # TODO area 参数还没给
                current_task = TaskSearch(10, 50)
                self.current_task = current_task
                current_task.start()
                self.task_state = TaskState.SEARCH
            else:
                pass

    def task_exec_state(self):
        # 搜索完毕
        if self.task_state == TaskState.SEARCH:
            # 搜索到目标
            if self.current_task.result:
                # TODO area 参数还没给
                current_task = TaskTrack(10)
                self.current_task = current_task
                current_task.start()
                self.task_state = TaskState.TRACK
            else:
                self.task_state = TaskState.HOVER
        elif self.task_state == TaskState.SEARCH:
            pass


if __name__ == "__main__":
    position_PID_gains = airsim.PositionControllerGains(airsim.PIDGains(0.25, 0.1, 0.1),
                                                        airsim.PIDGains(0.25, 0.1, 0.1),
                                                        airsim.PIDGains(0.25, 0.1, 0.1))
    vehicle_name1 = "Drone1"
    client1 = airsim.MultirotorClient()
    client1.confirmConnection()
    client1.setPositionControllerGains(position_PID_gains, vehicle_name1)
    client1.enableApiControl(True, vehicle_name1)
    drone1 = DroneControl(vehicle_name1, client1)

    vehicle_name2 = "Drone2"
    client2 = airsim.MultirotorClient()
    client2.confirmConnection()
    client2.setPositionControllerGains(position_PID_gains, vehicle_name2)
    client2.enableApiControl(True, vehicle_name2)
    drone2 = DroneControl(vehicle_name2, client2)

    drone1.start()
    print("drone1 start")
    drone2.start()
    print("drone2 start")
