# airsim_collaborative_tracking

本项目利用Airsim和UE4实现了一个无人机协同跟踪的demo，协同任务比较简单：一个大型场景下，每个无人机管控一块子区域。无人机跟随目标，一旦无人机超出管控区域，则判断目标的位置，并计算目标去了哪个无人机的区域，给其发送位置信息。接收无人机接收到信息，立即赶往目标位置，进行搜索和跟踪任务。

有用的文件：

`PIDcontroller.py`: PID 控制类

`drone_control.py`: 项目入口文件，也是无人机的顶层控制

`param_setting.py`: 各种全局参数（和超参数）

`setting.json`: 对应的配置文件

`t_task.py`: 实现各类无人机任务

`t_types.py`: 一些数据类型和类

`t_util.py`: 一些实用函数


other：无用文件
