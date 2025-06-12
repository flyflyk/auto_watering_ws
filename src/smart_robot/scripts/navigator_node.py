#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from smart_robot.msg import MoveToPlantAction, MoveToPlantResult, MoveToPlantFeedback
from tf.transformations import euler_from_quaternion

class NavigatorNode:
    def __init__(self):
        rospy.init_node('navigator_node')

        # --- 狀態定義 ---
        self.STATE_GOAL_SEEKING = 0
        self.STATE_WALL_FOLLOWING = 1
        self.current_state = self.STATE_GOAL_SEEKING

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.server = actionlib.SimpleActionServer('move_to_plant', MoveToPlantAction, self.execute, False)
        self.server.start()

        # --- 感測器數據 ---
        self.current_pos = Point()
        self.current_yaw = 0.0
        # 將雷射數據分區：右、右前、正前、左前、左
        self.regions = {
            'right': float('inf'),
            'fright': float('inf'),
            'front': float('inf'),
            'fleft': float('inf'),
            'left': float('inf'),
        }

        rospy.loginfo("Navigator Action Server with Bug Algorithm is ready.")

    def odom_callback(self, msg):
        self.current_pos = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def scan_callback(self, msg):
        # 建立一個輔助函數來安全地取最小值
        def get_safe_min(range_list):
            valid_ranges = [r for r in range_list if r > 0.0 and not math.isinf(r)]
            return min(valid_ranges) if valid_ranges else 10.0

        # 將360度的雷射數據分成5個區域
        self.regions = {
            'right':  get_safe_min(msg.ranges[285:325]),
            'fright': get_safe_min(msg.ranges[325:345]),
            'front':  get_safe_min(msg.ranges[0:15] + msg.ranges[345:360]),
            'fleft':  get_safe_min(msg.ranges[15:35]),
            'left':   get_safe_min(msg.ranges[35:75]),
        }

    def execute(self, goal):
        target_pos = goal.target_plant_position
        rospy.loginfo(f"Received goal to move to ({target_pos.x:.2f}, {target_pos.y:.2f})")
        
        self.current_state = self.STATE_GOAL_SEEKING # 每次新任務都重置狀態
        rate = rospy.Rate(10)
        feedback = MoveToPlantFeedback()
        result = MoveToPlantResult()

        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                result.success = False
                break
            
            # --- 核心狀態機邏輯 ---
            move_cmd = Twist()
            
            # 計算到目標的距離，如果已到達則成功
            dist_to_goal = math.sqrt((target_pos.x - self.current_pos.x)**2 + (target_pos.y - self.current_pos.y)**2)
            if dist_to_goal < 0.25:
                rospy.loginfo("Goal reached.")
                result.success = True
                break

            # 狀態一：尋找目標
            if self.current_state == self.STATE_GOAL_SEEKING:
                # 檢查是否需要切換到沿牆模式
                if self.regions['front'] < 0.6:
                    rospy.logwarn("Obstacle detected! Switching to WALL_FOLLOWING state.")
                    self.current_state = self.STATE_WALL_FOLLOWING
                else:
                    move_cmd = self.calculate_goal_seeking_cmd(target_pos)

            # 狀態二：沿牆行走
            elif self.current_state == self.STATE_WALL_FOLLOWING:
                # 檢查是否可以切換回尋找目標模式
                if self.is_path_to_goal_clear(target_pos):
                    rospy.loginfo("Path to goal is clear. Switching back to GOAL_SEEKING state.")
                    self.current_state = self.STATE_GOAL_SEEKING
                else:
                    move_cmd = self.calculate_wall_following_cmd()
            
            self.cmd_vel_pub.publish(move_cmd)

            feedback.current_robot_position = self.current_pos
            self.server.publish_feedback(feedback)
            rate.sleep()

        # 任務結束，停止機器人
        self.cmd_vel_pub.publish(Twist())
        if result.success:
            self.server.set_succeeded(result)

    def calculate_goal_seeking_cmd(self, target_pos):
        """計算朝向目標的移動指令"""
        move_cmd = Twist()
        angle_to_goal = math.atan2(target_pos.y - self.current_pos.y, target_pos.x - self.current_pos.x)
        angle_error = self.normalize_angle(angle_to_goal - self.current_yaw)

        if abs(angle_error) > 0.2: # 如果角度偏差大，先轉向
            move_cmd.linear.x = 0.1
            move_cmd.angular.z = 0.4 if angle_error > 0 else -0.4
        else: # 角度差不多了，前進
            move_cmd.linear.x = 0.25
            move_cmd.angular.z = 0.0
        return move_cmd

    def calculate_wall_following_cmd(self):
        """計算沿牆行走的移動指令 (保持在牆的右側)"""
        move_cmd = Twist()
        # 規則：
        # 1. 如果正前方太近，左轉
        if self.regions['front'] < 0.5:
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.5
        # 2. 否則，如果右前方也安全，可以稍微右轉去貼近牆壁
        elif self.regions['fright'] > 0.7:
            move_cmd.linear.x = 0.15
            move_cmd.angular.z = -0.4
        # 3. 否則，保持直行
        else:
            move_cmd.linear.x = 0.2
            move_cmd.angular.z = 0.0
        return move_cmd

    def is_path_to_goal_clear(self, target_pos):
        """檢查當前朝向目標的方向是否通暢"""
        angle_to_goal = math.atan2(target_pos.y - self.current_pos.y, target_pos.x - self.current_pos.x)
        angle_error = self.normalize_angle(angle_to_goal - self.current_yaw)
        
        # 如果目標就在正前方 (角度誤差小)，且正前方沒有障礙物
        if abs(angle_error) < math.pi / 4 and self.regions['front'] > 1.0:
            return True
        return False

    def normalize_angle(self, angle):
        """將角度正規化到 -pi 到 pi 之間"""
        if angle > math.pi:
            angle -= 2 * math.pi
        if angle < -math.pi:
            angle += 2 * math.pi
        return angle

if __name__ == '__main__':
    try:
        NavigatorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass