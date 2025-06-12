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

        # 等待 /get_moisture 服務
        rospy.loginfo("Waiting for get_moisture service...")
        rospy.wait_for_service('/get_moisture')
        self.get_moisture_service = rospy.ServiceProxy('/get_moisture', Trigger)
        
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
                
                 # 2. 呼叫服務來獲取濕度
            try:
                rospy.loginfo("Requesting moisture level...")
                moisture_response = self.get_moisture_service(TriggerRequest())
                if moisture_response.success:
                    current_moisture = float(moisture_response.message)
                    rospy.loginfo(f"Current moisture is {current_moisture:.2f}%.")
                    
                    # 3. 根據濕度決定是否澆水
                    if current_moisture < 50.0: # 假設低於50%就需要澆水
                        rospy.loginfo("Moisture is low. Triggering water pump...")
                        water_response = self.trigger_water_service(TriggerRequest())
                        if water_response.success:
                            rospy.loginfo("Watering complete. Waiting for 2 seconds.")
                            rospy.sleep(2)
                        else:
                            rospy.logwarn(f"Pump driver reported failure: {water_response.message}")
                    else:
                        rospy.loginfo("Moisture is sufficient. Skipping watering.")

                else:
                    rospy.logwarn("Failed to get moisture reading.")

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
