#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Trigger, TriggerRequest
from smart_robot.srv import GetMoisture

class ManagerNode:
    def __init__(self):
        rospy.init_node('manager_node')
        rospy.loginfo("Manager Node (move_base version) starting...")

        # 獲取參數
        plant_locations_raw = rospy.get_param("~plants", [])
        self.charging_station_pos = rospy.get_param("~charging_station/position", None)

        if not plant_locations_raw or not self.charging_station_pos:
            rospy.logerr("Plant or charging station locations not found on parameter server.")
            rospy.signal_shutdown("Missing required parameters")
            return
        
        self.plant_locations = self.sort_plants_by_distance(plant_locations_raw)
        
        rospy.loginfo("Plant locations sorted by distance:")
        for plant in self.plant_locations:
            rospy.loginfo(f"- {plant['name']} at distance {plant.get('distance', 0):.2f}m")

        # 連接到 move_base Action Server
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_client.wait_for_server()
        rospy.loginfo("move_base action server connected.")

        # 等待服務
        rospy.loginfo("Waiting for services...")
        rospy.wait_for_service('/trigger_water')
        self.trigger_water_service = rospy.ServiceProxy('/trigger_water', Trigger)
        rospy.wait_for_service('/get_moisture')
        self.get_moisture_service = rospy.ServiceProxy('/get_moisture', GetMoisture)
        
        rospy.loginfo("Manager Node is ready.")

    def sort_plants_by_distance(self, plants):
        start_pos = self.charging_station_pos
        for plant in plants:
            plant_pos = plant['position']
            distance = math.sqrt((plant_pos['x'] - start_pos['x'])**2 + (plant_pos['y'] - start_pos['y'])**2)
            plant['distance'] = distance
        return sorted(plants, key=lambda p: p['distance'])

    def run_watering_mission(self):
        rospy.loginfo("Starting watering mission with move_base...")
        for plant in self.plant_locations:
            rospy.loginfo(f"Next target: {plant['name']} at {plant['position']}")
            
            goal = self.create_move_base_goal(plant['position'])
            self.move_client.send_goal(goal)
            
            finished_in_time = self.move_client.wait_for_result(rospy.Duration(120.0)) # 增加超時時間

            if not finished_in_time:
                self.move_client.cancel_goal()
                rospy.logwarn(f"Timed out achieving goal for {plant['name']}")
            elif self.move_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"Successfully arrived at {plant['name']}.")
                self.check_and_water()
            else:
                rospy.logwarn(f"Failed to move to {plant['name']}. Status: {self.move_client.get_goal_status_text()}")
        
        self.return_to_base()

    def create_move_base_goal(self, position_dict):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(**position_dict)
        q = quaternion_from_euler(0, 0, 0) # 讓機器人朝向預設方向
        goal.target_pose.pose.orientation = Quaternion(*q)
        return goal

    def check_and_water(self):
        try:
            rospy.loginfo("Requesting moisture level...")
            moisture_response = self.get_moisture_service()
            if moisture_response.success:
                current_moisture = moisture_response.moisture_level
                rospy.loginfo(f"Current moisture is {current_moisture:.2f}%.")
                if current_moisture < 50.0:
                    rospy.loginfo("Moisture is low. Triggering water pump...")
                    self.trigger_water_service(TriggerRequest())
                    rospy.loginfo("Watering complete. Waiting for 2 seconds.")
                    rospy.sleep(2)
                else:
                    rospy.loginfo("Moisture is sufficient. Skipping watering.")
            else:
                rospy.logwarn("Failed to get moisture reading from sensor.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def return_to_base(self):
        rospy.loginfo("All plants visited. Returning to charging station.")
        goal = self.create_move_base_goal(self.charging_station_pos)
        self.move_client.send_goal(goal)
        self.move_client.wait_for_result(rospy.Duration(120.0))
        if self.move_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Arrived at charging station. Mission complete.")
        else:
            rospy.logerr("Failed to return to charging station.")

if __name__ == '__main__':
    try:
        manager = ManagerNode()
        rospy.sleep(1)
        manager.run_watering_mission()
    except rospy.ROSInterruptException:
        pass