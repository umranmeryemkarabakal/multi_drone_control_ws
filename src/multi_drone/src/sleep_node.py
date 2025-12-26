#!/usr/bin/env python
import rospy
from time import sleep

def sleep_node():
    rospy.init_node('sleep_node', anonymous=True)
    sleep_time = rospy.get_param("sleep_time", 2)  # Parametreyi al
    rospy.loginfo(f"Sleeping for {sleep_time} seconds")
    sleep(sleep_time)

if __name__ == '__main__':
    try:
        sleep_node()
    except rospy.ROSInterruptException:
        pass
