#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Trigger, TriggerRequest, Empty
from smart_robot.srv import GetMoisture

class ManagerNode:
    def __init__(self):
        rospy.init_node('manager_node')
        rospy.loginfo("Manager Node (move_base version) starting...")
        self.tf_listener = tf.TransformListener()

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
        rospy.loginfo("Waiting for /move_base/clear_costmaps service...")
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        rospy.loginfo("/move_base/clear_costmaps service connected.")
        
        rospy.loginfo("Manager Node is ready.")

    def sort_plants_by_distance(self, plants):
        start_pos = self.charging_station_pos
        for plant in plants:
            plant_pos = plant['position']
            distance = math.sqrt((plant_pos['x'] - start_pos['x'])**2 + (plant_pos['y'] - start_pos['y'])**2)
            plant['distance'] = distance
        return sorted(plants, key=lambda p: p['distance'])

    def run_watering_mission(self):
        rospy.loginfo("Waiting for navigation stack to be ready by sending a dummy goal...")
        
        # 1. 獲取當前位置作為熱身目標
        try:
            self.tf_listener.waitForTransform("map", "base_footprint", rospy.Time(), rospy.Duration(10.0))
            (trans, rot) = self.tf_listener.lookupTransform("map", "base_footprint", rospy.Time(0))
            warm_up_pos = {'x': trans[0], 'y': trans[1]}
            rospy.loginfo(f"Got current position for warm-up: {warm_up_pos}")
        except (tf.Exception) as e:
            rospy.logerr(f"Cannot get robot position for warm-up. Aborting mission. Error: {e}")
            return # 如果連位置都獲取不到，直接終止任務

        # 2. 發送熱身目標
        warm_up_goal = MoveBaseGoal()
        warm_up_goal.target_pose.header.frame_id = "map"
        warm_up_goal.target_pose.header.stamp = rospy.Time.now()
        warm_up_goal.target_pose.pose.position = Point(**warm_up_pos)
        warm_up_goal.target_pose.pose.orientation.w = 1.0

        self.move_client.send_goal(warm_up_goal)
        finished_in_time = self.move_client.wait_for_result(rospy.Duration(15.0)) # 給15秒時間來完成這個簡單的任務

        if not finished_in_time or self.move_client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            rospy.logerr("Navigation stack warm-up failed. The robot might be stuck or localization is not ready. Aborting mission.")
            return
        
        rospy.loginfo("Navigation stack is ready! Starting watering mission...")
        current_pos = warm_up_pos

        for plant in self.plant_locations:
            rospy.loginfo(f"Next target: {plant['name']} at {plant['position']}")
            goal = self.create_move_base_goal(plant['position'], current_pos)
            self.move_client.send_goal(goal)
            finished_in_time = self.move_client.wait_for_result(rospy.Duration(120.0))

            if not finished_in_time:
                self.move_client.cancel_goal()
                rospy.logwarn(f"Timed out achieving goal for {plant['name']}")
            elif self.move_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"Successfully arrived at {plant['name']}.")
                self.check_and_water()
                # 更新當前位置，為下一個目標做準備
                current_pos = plant['position']
                rospy.loginfo("Clearing costmaps before next goal...")
                try:
                    self.clear_costmaps_service()
                    rospy.loginfo("Costmaps cleared successfully.")
                    rospy.sleep(1.0) # 等待1秒，讓清除操作生效
                except rospy.ServiceException as e:
                    rospy.logerr(f"Failed to clear costmaps: {e}")
            else:
                rospy.logwarn(f"Failed to move to {plant['name']}. Status: {self.move_client.get_goal_status_text()}")
        
        self.return_to_base(current_pos)

    def create_move_base_goal(self, target_position_dict, current_position_dict):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position = Point(**target_position_dict)
        
        delta_x = target_position_dict['x'] - current_position_dict['x']
        delta_y = target_position_dict['y'] - current_position_dict['y']
        
        yaw = math.atan2(delta_y, delta_x)
        
        q = quaternion_from_euler(0, 0, yaw)
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

    def return_to_base(self, current_position_dict):
        rospy.loginfo("All plants visited. Returning to charging station.")
        goal = self.create_move_base_goal(self.charging_station_pos, current_position_dict)
        self.move_client.send_goal(goal)
        self.move_client.wait_for_result(rospy.Duration(120.0))
        if self.move_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Arrived at charging station. Mission complete.")
        else:
            rospy.logerr("Failed to return to charging station.")

if __name__ == '__main__':
    try:
        manager = ManagerNode()
        manager.run_watering_mission()
    except rospy.ROSInterruptException:
        pass