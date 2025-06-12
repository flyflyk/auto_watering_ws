#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import random
from std_srvs.srv import Trigger, TriggerResponse

def handle_get_moisture(req):
    """
    當服務被呼叫時，這個函數會被執行。
    它會模擬一次感測，並將結果返回。
    """
    # 模擬一個 20% 到 80% 之間的濕度值
    moisture_level = random.uniform(20.0, 80.0)
    rospy.loginfo(f"Sensed moisture: {moisture_level:.2f}%")
    
    return TriggerResponse(success=True, message=str(moisture_level))

def moisture_sensor_node():
    rospy.init_node('moisture_sensor_node')
    
    # 建立一個名為 /get_moisture 的服務，類型為 Trigger
    s = rospy.Service('/get_moisture', Trigger, handle_get_moisture)
    
    rospy.loginfo("Moisture sensor service is ready.")
    rospy.spin()

if __name__ == '__main__':
    try:
        moisture_sensor_node()
    except rospy.ROSInterruptException:
        pass