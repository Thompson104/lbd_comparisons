#!/usr/bin/env python
import subprocess
import time

if __name__ == "__main__":
    subprocess.check_call(['rosrun','ros_bag_loader','ros_bag_loader.py'])
    time.sleep(5)
    subprocess.call(['rosrun', 'play_back', 'play_back.py'])
