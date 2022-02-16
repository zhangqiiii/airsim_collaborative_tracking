import math
import airsim

# vehicle_name = "Drone1"
camera_name = "high_res"
pi = math.pi
camera_width = 600
camera_height = 400
max_speed = 6
image_type = airsim.ImageType.Scene


# 区域划分
all_area = [[(-394, 0), (-397, 397), (394, 397), (394, 0)],
            [(-394, 0), (-393, -374), (385, -374), (394, 0)]]
# all_area = [[(-394, -374), (-397, 397), (394, 397), (394, -374)],
#             [(-394, -84), (-393, -374), (385, -374), (394, -84)]]

init_position = [(0, 160, -2), (0, -160, -2)]
