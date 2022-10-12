import os
import time
import subprocess


BUILD_NAME = os.getcwd().split(os.sep)[-1]
#HEX = (f"rsync build/{BUILD_NAME}.hex pi@192.168.1.238:./CAN_FLASH/{BUILD_NAME}/")
#CRC = (f"rsync build/{BUILD_NAME}_CRC.hex pi@192.168.1.238:./CAN_FLASH/{BUILD_NAME}/")
HEX = (f"rsync build/{BUILD_NAME}.hex pi@raspberrypi:./CAN_FLASH/{BUILD_NAME}/")
CRC = (f"rsync build/{BUILD_NAME}_CRC.hex pi@raspberrypi:./CAN_FLASH/{BUILD_NAME}/")
REMOTE_LOAD = (f"rsync PYTHON/CAN-FLASH-PI.py pi@raspberrypi:./CAN_FLASH/{BUILD_NAME}/")
PI_LOAD = (f"rsync PYTHON/CAN-FLASH-OLED.py pi@raspberrypi:./CAN_FLASH/{BUILD_NAME}/")


print(HEX)
output = subprocess.check_output(HEX, shell=True)
print(output)
print(CRC)
output = subprocess.check_output(CRC, shell=True)
print(REMOTE_LOAD)
output = subprocess.check_output(REMOTE_LOAD, shell=True)
print(output)
print(PI_LOAD)
output = subprocess.check_output(PI_LOAD, shell=True)
print(output)