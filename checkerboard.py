import airsim
import os
import numpy as np
import cv2
import time

"""
This file is used to get the images of the checkerboard in order to calibrate the camera of the drone for gate
detection.
The code utilizes the created checkerboard environment to capture images using the get image function of AirSim.
The files are saved in a separate directory for calibration at a later stage.
"""

# Create the multirotor client and enable API control
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Takeoff
print("Taking off...")
client.takeoffAsync().join()
time.sleep(1)

# Move the drone to face the checkerboard and call additional move functions to change the orientation. This helps to
# capture multiple angles of the checkerboard.
client.moveByRollPitchYawZAsync(0, 0, np.radians(-180), -3, duration=10, vehicle_name="drone_1").join()
client.moveByRollPitchYawZAsync(np.radians(180), 0, np.radians(-180), -10, duration=1.5, vehicle_name="drone_1").join()

# Capture the image
raw_image = client.simGetImage("0", airsim.ImageType.Scene)
if raw_image is None:
    raise RuntimeError("Image capture failed: no data returned")

# Convert binary string to a numpy array
png_array = np.frombuffer(raw_image, dtype=np.uint8)
# Decode PNG
img_rgb = cv2.imdecode(png_array, cv2.IMREAD_COLOR)
if img_rgb is None:
    raise RuntimeError("PNG decoding failed")

# Save to disk
filename = "D:\Purdue\Spring 2025\VIP 37920\Gate_Images2\image2.png"
cv2.imwrite(filename, img_rgb)
print(f"Image saved as {filename}")

client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)
