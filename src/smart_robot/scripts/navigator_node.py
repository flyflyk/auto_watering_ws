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

        self.load_params()

        # --- 狀態定義 ---
        self.STATE_GOAL_SEEKING = 0
        self.STATE_WALL_FOLLOWING = 1
        self.current_state = self.STATE_GOAL_SEEKING

        # --- ROS 通訊 ---
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.server = actionlib.SimpleActionServer('move_to_plant', MoveToPlantAction, self.execute, False)
        self.server.start()

        # --- 感測器數據與狀態 ---
        self.current_pos = Point()
        self.current_yaw = 0.0
        self.regions = {'right': 10.0, 'fright': 10.0, 'front': 10.0, 'fleft': 10.0, 'left': 10.0}
        self.angular_p_gain = rospy.get_param('~angular_p_gain', 2.0)
        self.hit_point = None
        self.min_dist_to_goal = float('inf')

        rospy.loginfo("Navigator Action Server with Bug Algorithm is ready.")

    def load_params(self):
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.25)
        self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 0.6)
        self.path_clear_threshold = rospy.get_param('~path_clear_threshold', 1.0)
        
        self.forward_speed = rospy.get_param('~forward_speed', 0.25)
        self.slow_forward_speed = rospy.get_param('~slow_forward_speed', 0.15)
        self.turn_speed = rospy.get_param('~turn_speed', 0.4)
        self.wall_follow_turn_speed = rospy.get_param('~wall_follow_turn_speed', 0.5)

        self.wf_front_dist = rospy.get_param('~wall_follow/front_dist_trigger', 0.5)
        self.wf_side_dist = rospy.get_param('~wall_follow/side_dist_target', 0.7)

        self.angle_tolerance = rospy.get_param('~angle_tolerance', 0.2)
        self.path_clear_angle = rospy.get_param('~path_clear_angle', math.pi / 4) # 45 degrees
        rospy.loginfo("Navigator parameters loaded.")

    def odom_callback(self, msg):
        self.current_pos = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def scan_callback(self, msg):
        def get_safe_min(range_list):
            valid_ranges = [r for r in range_list if r > 0.0 and not math.isinf(r)]
            return min(valid_ranges) if valid_ranges else 10.0

        num_ranges = len(msg.ranges)
        if num_ranges != 90:
             rospy.logwarn_throttle(5, f"Expected 90 laser points, but got {num_ranges}. Check URDF configuration.")
             return

        self.regions = {
            'right':  float('inf'),
            'fright': get_safe_min(msg.ranges[0:30]),
            'front':  get_safe_min(msg.ranges[30:60]),
            'fleft':  get_safe_min(msg.ranges[60:90]),
            'left':   float('inf'),
        }

    def execute(self, goal):
        target_pos = goal.target_plant_position
        rospy.loginfo(f"Executing goal to move to ({target_pos.x:.2f}, {target_pos.y:.2f})")
        rate = rospy.Rate(60)
        result = MoveToPlantResult()
        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                self.server.set_preempted(); result.success = False; break
            dist_to_goal = math.sqrt((target_pos.x - self.current_pos.x)**2 + (target_pos.y - self.current_pos.y)**2)
            if dist_to_goal < self.goal_tolerance:
                rospy.loginfo("Goal reached."); result.success = True; break
            
            # 這裡我們甚至可以暫時忽略障礙物，專心驗證移動
            move_cmd = self.calculate_goal_seeking_cmd(target_pos)
            self.cmd_vel_pub.publish(move_cmd)

            # 為了除錯，我們打印出收到的 odom 和發出的 cmd_vel
            rospy.loginfo_throttle(1.0, f"Odom: [x={self.current_pos.x:.2f}, y={self.current_pos.y:.2f}], Cmd: [lin={move_cmd.linear.x:.2f}, ang={move_cmd.angular.z:.2f}]")
            
            rate.sleep()
        self.cmd_vel_pub.publish(Twist())
        if result.success: 
            self.server.set_succeeded(result)

    def calculate_goal_seeking_cmd(self, target_pos):
        move_cmd = Twist()
        angle_to_goal = math.atan2(target_pos.y - self.current_pos.y, target_pos.x - self.current_pos.x)
        angle_error = self.normalize_angle(angle_to_goal - self.current_yaw)
        angular_speed = self.angular_p_gain * angle_error
        clamped_angular_speed = max(-self.turn_speed, min(self.turn_speed, angular_speed))

        if abs(angle_error) > 0.35:
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = clamped_angular_speed
        else:
            move_cmd.linear.x = self.forward_speed
            move_cmd.angular.z = clamped_angular_speed
            
        return move_cmd
    
    def calculate_wall_following_cmd(self):
        move_cmd = Twist()
        # 1. 如果正前方或右前方太近，說明要撞牆或入彎，必須左轉
        if self.regions['front'] < self.wf_front_dist or self.regions['fright'] < self.wf_front_dist:
            rospy.logwarn_throttle(1, "Wall Following: Turning left to avoid collision.")
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = self.wall_follow_turn_speed
        # 2. 如果右邊太空曠了，可以稍微右轉去貼近牆壁
        elif self.regions['right'] > self.wf_side_dist:
            rospy.logwarn_throttle(1, "Wall Following: Turning right to find wall.")
            move_cmd.linear.x = self.slow_forward_speed
            move_cmd.angular.z = -self.turn_speed
        # 3. 距離正好，沿牆直行
        else:
            rospy.logwarn_throttle(1, "Wall Following: Moving straight.")
            move_cmd.linear.x = self.forward_speed
            move_cmd.angular.z = 0.0
        return move_cmd

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

if __name__ == '__main__':
    try:
        NavigatorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass