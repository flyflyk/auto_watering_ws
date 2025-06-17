#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Trigger, TriggerRequest
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
        
        rospy.loginfo("Manager Node is ready.")


    def sort_plants_by_distance(self, plants):
        start_pos = self.charging_station_pos
        for plant in plants:
            plant_pos = plant['position']
            distance = math.sqrt((plant_pos['x'] - start_pos['x'])**2 + (plant_pos['y'] - start_pos['y'])**2)
            plant['distance'] = distance
        return sorted(plants, key=lambda p: p['distance'])

    def run_watering_mission(self):
        rospy.loginfo("Waiting for the AMCL localization to be ready...")
        rospy.loginfo("This is done by waiting for the 'map' to 'odom' transform.")
        
        try:
            self.tf_listener.waitForTransform("map", "odom", rospy.Time(), rospy.Duration(60.0))
            rospy.loginfo("AMCL is ready! The 'map' -> 'odom' transform is available.")
        except (tf.Exception) as e:
            rospy.logerr(f"Could not get transform from 'map' to 'odom' after 60 seconds. AMCL might not be working. Aborting mission. Error: {e}")
            return
        
        rospy.loginfo("Giving AMCL 10 seconds to converge...")
        rospy.sleep(10.0)
        rospy.loginfo("Navigation stack is ready! Starting watering mission...")
        
        # 獲取可靠的當前位置作為起點
        try:
            (trans, rot) = self.tf_listener.lookupTransform("map", "base_footprint", rospy.Time(0))
            current_pos = {'x': trans[0], 'y': trans[1]}
        except (tf.Exception) as e:
            rospy.logerr(f"CRITICAL: Failed to get initial robot position even after AMCL is ready. Error: {e}")
            return
            
        for plant in self.plant_locations:
            rospy.loginfo(f"Next target: {plant['name']} at {plant['position']}")
            goal = self.create_move_base_goal(plant['position'], current_pos)
            self.move_client.send_goal(goal)
            finished_in_time = self.move_client.wait_for_result(rospy.Duration(300.0))

            if not finished_in_time:
                self.move_client.cancel_goal()
                rospy.logwarn(f"Timed out achieving goal for {plant['name']}")
            elif self.move_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"Successfully arrived at {plant['name']}.")
                self.check_and_water()
                
                self.execute_backup_goal() 

                try:
                    self.tf_listener.waitForTransform("map", "base_footprint", rospy.Time(), rospy.Duration(4.0))
                    (trans, rot) = self.tf_listener.lookupTransform("map", "base_footprint", rospy.Time(0))
                    current_pos = {'x': trans[0], 'y': trans[1]}
                    rospy.loginfo(f"Position after backup: {current_pos}")
                except (tf.Exception) as e:
                    rospy.logerr(f"CRITICAL: Could not get robot position after backup: {e}. Aborting mission to prevent unpredictable behavior.")
                    return 
            else:
                rospy.logwarn(f"Failed to move to {plant['name']}. Status: {self.move_client.get_goal_status_text()}")
        
        self.return_to_base(current_pos)
    
    def execute_backup_goal(self, distance=-1.5):
        rospy.loginfo(f"Executing backup maneuver using move_base for {abs(distance)}m.")
        
        backup_goal = MoveBaseGoal()
        backup_goal.target_pose.header.frame_id = "base_footprint"
        backup_goal.target_pose.header.stamp = rospy.Time()
        
        backup_goal.target_pose.pose.position.x = distance
        backup_goal.target_pose.pose.orientation.w = 1.0
        
        self.move_client.send_goal(backup_goal)
        
        finished_in_time = self.move_client.wait_for_result(rospy.Duration(20.0))
        
        if not finished_in_time:
            self.move_client.cancel_goal()
            rospy.logwarn("Timed out during backup maneuver.")
        elif self.move_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Backup maneuver complete.")
        else:
            rospy.logwarn(f"Backup maneuver failed. Status: {self.move_client.get_goal_status_text()}")
        
        rospy.sleep(1.0)

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
        self.move_client.wait_for_result(rospy.Duration(300.0))
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