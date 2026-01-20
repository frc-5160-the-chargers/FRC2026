"""

"""
import sys
import time

from pydualsense import pydualsense
from ntcore import NetworkTableInstance

is_sim = "-sim" in sys.argv

def main():
    print("Initializing DualSense API...")
    controller = pydualsense()
    controller.init()
    try:
        print("Initializing NetworkTables Client....")
        inst = NetworkTableInstance.getDefault()
        if is_sim:
            inst.setServer("127.0.0.1", 5810)
        else:
            inst.setServerTeam(5160)
        inst.startClient4("PS5 Controller Rumble Client")
        print("NetworkTables client has started.")
        left_rumble_sub = inst.getIntegerTopic("ControllerRumble/LeftRequest").subscribe(0)
        right_rumble_sub = inst.getIntegerTopic("ControllerRumble/RightRequest").subscribe(0)
        time.sleep(5)
        while True:
            controller.setLeftMotor(left_rumble_sub.get())
            controller.setRightMotor(right_rumble_sub.get())
    finally:
        controller.close()

if __name__ == "__main__":
    main()