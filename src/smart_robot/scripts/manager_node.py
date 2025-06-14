#!/usr/bin/python3
# -*- coding: utf-8 -*-

import math
import rospy
import actionlib
from geometry_msgs.msg import Point
from smart_robot.msg import MoveToPlantAction, MoveToPlantGoal
from smart_robot.srv import GetMoisture
from std_srvs.srv import Trigger, TriggerRequest

class ManagerNode:
    def __init__(self):
        rospy.init_node('manager_node')
        rospy.loginfo("Manager Node starting...")

        # Get parameters
        plant_locations_raw = rospy.get_param("~plants", [])
        self.charging_station_pos = rospy.get_param("~charging_station/position", None)

        if not plant_locations_raw or not self.charging_station_pos:
            rospy.logerr("Plant or charging station locations not found on parameter server.")
            rospy.signal_shutdown("Missing required parameters")
            return
        
        self.plant_locations = self.sort_plants_by_distance(plant_locations_raw)
        rospy.loginfo("Plant locations sorted by distance:")
        for plant in self.plant_locations:
            rospy.loginfo(f"- {plant['name']} at distance {plant['distance']:.2f}m")

        # Action Client
        self.move_client = actionlib.SimpleActionClient('move_to_plant', MoveToPlantAction)
        rospy.loginfo("Waiting for move_to_plant action server...")
        self.move_client.wait_for_server()

        # Service Proxies
        rospy.loginfo("Waiting for services...")
        rospy.wait_for_service('/trigger_water')
        self.trigger_water_service = rospy.ServiceProxy('/trigger_water', Trigger)
        
        rospy.wait_for_service('/get_moisture')
        self.get_moisture_service = rospy.ServiceProxy('/get_moisture', GetMoisture)
        
        rospy.loginfo("Manager Node is ready.")
    
    def sort_plants_by_distance(self, plants):
        start_pos = self.charging_station_pos
        
        # 1. 計算每個盆栽的距離並儲存
        for plant in plants:
            plant_pos = plant['position']
            distance = math.sqrt((plant_pos['x'] - start_pos['x'])**2 + (plant_pos['y'] - start_pos['y'])**2)
            plant['distance'] = distance
        
        # 2. 對列表進行排序
        sorted_plants = sorted(plants, key=lambda p: p['distance'])
        
        return sorted_plants

    def run_watering_mission(self):
        rospy.loginfo("Starting watering mission...")
        
        for plant in self.plant_locations:
            rospy.loginfo(f"Next target: {plant['name']} at {plant['position']}")
            
            goal = MoveToPlantGoal(target_plant_position=Point(**plant['position']))
            self.move_client.send_goal(goal)
            self.move_client.wait_for_result(rospy.Duration(60.0))

            if self.move_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"Successfully arrived at {plant['name']}.")
                self.check_and_water()
            else:
                rospy.logwarn(f"Failed to move to {plant['name']} (State: {self.move_client.get_goal_status_text()}). Skipping.")

        self.return_to_base()

    def check_and_water(self):
        try:
            rospy.loginfo("Requesting moisture level...")
            moisture_response = self.get_moisture_service() 
            
            if moisture_response.success:
                current_moisture = moisture_response.moisture_level 
                rospy.loginfo(f"Current moisture is {current_moisture:.2f}%.")
                
                if current_moisture < 50.0:
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
                rospy.logwarn("Failed to get moisture reading from sensor.")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def return_to_base(self):
        rospy.loginfo("All plants visited. Returning to charging station.")
        goal = MoveToPlantGoal(target_plant_position=Point(**self.charging_station_pos))
        self.move_client.send_goal(goal)
        self.move_client.wait_for_result(rospy.Duration(120.0))

        if self.move_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Arrived at charging station. Mission complete.")
        else:
            rospy.logerr(f"Failed to return to charging station (State: {self.move_client.get_goal_status_text()}).")

if __name__ == '__main__':
    try:
        manager = ManagerNode()
        rospy.sleep(2)
        manager.run_watering_mission()
    except rospy.ROSInterruptException:
        pass
    except rospy.exceptions.ROSException as e:
        rospy.logerr(f"Shutdown requested before initialization: {e}")