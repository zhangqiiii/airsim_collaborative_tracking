import math
import airsim
import threading

# vehicle_name = "Drone1"
camera_name = "high_res"
pi = math.pi
camera_width = 600
camera_height = 400
max_speed = 6
image_type = airsim.ImageType.Scene

g_position_speed = 5

# search
g_search_min = 15
g_search_max = 35
g_track_height = 10
g_add_height = 10

# tracking
g_speed_PID = (0.002, 0.01, 0.01, 15)
g_yaw_PID = (0.08, 0.01, 0.05, math.radians(8))

# avoid obstacle
g_warn_dist = 5.0
g_danger_dist = 3
g_danger_add_height = 1.0

lock = threading.Lock()
# 区域划分
# all_area = [[(-394, 0), (-397, 397), (394, 397), (394, 0)],
#             [(-394, 0), (-393, -374), (385, -374), (394, 0)]]
all_area = [[(217, -150), (0, -150), (0, 150), (217, 150)],
            [(-183, -150), (0, -150), (0, 150), (-183, 150)]]

# init_position = [(0, 200, -2), (0, -200, -2)]
init_position = [(145, -37, -0.5), (-4.6, 67.5, -0.5)]

