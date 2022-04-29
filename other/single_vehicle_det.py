import airsim

import numpy as np
import os
import tempfile
import pprint
import cv2

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()

client.enableApiControl(True, "Drone1")

# airsim.wait_key('Press any key to takeoff')
print("Taking off...")
client.armDisarm(True)
client.takeoffAsync().join()

# airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
client.moveToPositionAsync(0, 0, -10, 5).join()

client.hoverAsync().join()

camera_name = "high_res"
image_type = airsim.ImageType.Scene

client.simSetDetectionFilterRadius(camera_name, image_type, 200 * 100)  # in [cm]
client.simAddDetectionFilterMeshName(camera_name, image_type, "Sphere")

state = client.getMultirotorState()
s = pprint.pformat(state)
print("state: %s" % s)

cv2.namedWindow("AirSim", cv2.WINDOW_KEEPRATIO)
while True:
    if input == 'q':
        break
    rawImage = client.simGetImage(camera_name, image_type)
    if not rawImage:
        continue
    png = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_UNCHANGED)
    objs = client.simGetDetections(camera_name, image_type)
    if objs:
        for obj in objs:
            cv2.rectangle(png, (int(obj.box2D.min.x_val), int(obj.box2D.min.y_val)),
                          (int(obj.box2D.max.x_val), int(obj.box2D.max.y_val)),
                          (255, 0, 0), 2)
    cv2.imshow("AirSim", png)

cv2.destroyAllWindows()

airsim.wait_key('Press any key to reset to original state')

client.reset()
client.armDisarm(False)

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)


