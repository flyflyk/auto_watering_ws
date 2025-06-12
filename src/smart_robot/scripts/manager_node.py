#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from geometry_msgs.msg import Point
from smart_robot.msg import MoveToPlantAction, MoveToPlantGoal
from std_srvs.srv import Trigger, TriggerRequest

class ManagerNode:
    def __init__(self):
        rospy.init_node('manager_node')
        rospy.loginfo("Manager Node starting...")

        # 獲取參數伺服器中的植物位置列表
        self.plant_locations = rospy.get_param("~plants", [])
        if not self.plant_locations:
            rospy.logerr("No plant locations found on parameter server.")
            return

        # 建立 Action Client，用於與 navigator_node 溝通
        self.move_client = actionlib.SimpleActionClient('move_to_plant', MoveToPlantAction)
        rospy.loginfo("Waiting for move_to_plant action server...")
        self.move_client.wait_for_server()

        # 等待 /trigger_water 服務
        rospy.loginfo("Waiting for trigger_water service...")
        rospy.wait_for_service('/trigger_water')
        self.trigger_water_service = rospy.ServiceProxy('/trigger_water', Trigger)
        
        rospy.loginfo("Manager Node is ready.")

    def run_watering_mission(self):
        rospy.loginfo("Starting watering mission...")
        for plant in self.plant_locations:
            rospy.loginfo(f"Next target: {plant['name']} at {plant['position']}")

            # 1. 發送移動目標給 Navigator
            goal = MoveToPlantGoal()
            goal.target_plant_position = Point(**plant['position'])
            self.move_client.send_goal(goal)
            
            # 等待移動任務完成
            self.move_client.wait_for_result(rospy.Duration(60.0)) # 60秒超時

            if self.move_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"Successfully arrived at {plant['name']}.")
                
                # 2. 觸發澆水
                try:
                    rospy.loginfo("Triggering water pump...")
                    response = self.trigger_water_service(TriggerRequest())
                    if response.success:
                        rospy.loginfo("Watering complete. Waiting for 2 seconds.")
                        rospy.sleep(2) # 模擬澆水時間
                    else:
                        rospy.logwarn(f"Pump driver reported failure: {response.message}")
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call failed: {e}")
            else:
                rospy.logwarn(f"Failed to move to {plant['name']}. Skipping.")
        
        rospy.loginfo("All plants have been visited. Mission complete.")

if __name__ == '__main__':
    try:
        manager = ManagerNode()
        # 給一點時間讓其他節點初始化
        rospy.sleep(1)
        manager.run_watering_mission()
    except rospy.ROSInterruptException:
        pass
