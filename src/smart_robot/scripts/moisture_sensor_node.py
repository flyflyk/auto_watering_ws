#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import random
from smart_robot.srv import GetMoisture, GetMoistureResponse

def handle_get_moisture(req):
    moisture_level = random.uniform(20.0, 80.0)
    rospy.loginfo(f"Sensed moisture: {moisture_level:.2f}%")
    
    return GetMoistureResponse(success=True, moisture_level=moisture_level)

def moisture_sensor_node():
    rospy.init_node('moisture_sensor_node')
    s = rospy.Service('/get_moisture', GetMoisture, handle_get_moisture)
    
    rospy.loginfo("Moisture sensor service is ready.")
    rospy.spin()

if __name__ == '__main__':
    try:
        moisture_sensor_node()
    except rospy.ROSInterruptException:
        pass