#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from std_srvs.srv import Trigger, TriggerResponse

def handle_trigger_water(req):
    rospy.loginfo("Pump activated! Watering for 1 second...")
    # 在真實世界中，這裡會是控制 GPIO 的程式碼
    rospy.sleep(1.0) 
    rospy.loginfo("Watering finished.")
    return TriggerResponse(success=True, message="Watering was successful.")

def pump_driver_node():
    rospy.init_node('pump_driver_node')
    # 建立一個名為 /trigger_water 的服務
    s = rospy.Service('/trigger_water', Trigger, handle_trigger_water)
    rospy.loginfo("Pump driver is ready to receive watering commands.")
    rospy.spin()

if __name__ == "__main__":
    pump_driver_node()
