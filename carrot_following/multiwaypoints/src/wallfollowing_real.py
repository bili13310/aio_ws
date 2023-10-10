#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import *
from time import *

LEFT = "LEFT"
RIGHT = "RIGHT"


class Limo_wall_following:
    def __init__(self, direction):
        rospy.init_node("laser_scan_node")
        # rospy.Subscriber("limo/scan", LaserScan, self.laser_callback)
        rospy.Subscriber("/scan_filtered", LaserScan, self.laser_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
        self.cmd_vel_msg = Twist()

        self.condition = None
        self.speed = 0
        self.angle = 0

        self.lidar_flag = False
        self.is_scan = False

        self.distance = []
        self.obstacle_data_range = []
        self.obstacle_data_idx = []

        self.DIRECTION = direction
        self.default_speed = 0.20
        self.default_angle = 0.2
        self.scan_dist = 0.8   # 0.8
        self.offset = 0.5       # 0.5

        self.msg = None

    def laser_callback(self, msg):
        self.msg = msg
        self.is_scan = True
        print("callback")

    def LiDAR_scan(self):
        print("LiDAR_scan")
        if self.lidar_flag == False:
            self.degrees = [
                (self.msg.angle_min + (i * self.msg.angle_increment)) * 180 / pi
                for i, data in enumerate(self.msg.ranges)
            ]
            self.lidar_flag = True

        # 원하는 각도 내에서 원하는 거리값 이내 값 뽑기
        for i, data in enumerate(self.msg.ranges):
            if -90 < self.degrees[i] < 90 and 0 < self.msg.ranges[i] <= self.scan_dist:
                self.distance.append(data)

        # 전방 디택팅 각도
        for i, data in enumerate(self.msg.ranges):
            if -5 < self.degrees[i] < -1 and 0 < self.msg.ranges[i] < self.scan_dist:
                self.obstacle_data_idx.append(i)
                self.obstacle_data_range.append(data)

    def judge_distance(self):
        if 0 <= len(self.obstacle_data_range) <= 4:
            self.condition = "forward"
            self.speed = self.default_speed
            if len(self.distance) > 0:
                if min(self.distance) < self.scan_dist - 0.6:
                    self.condition = "close"

                elif (
                    self.scan_dist - self.offset <= min(self.distance) <= self.scan_dist+self.offset
                ):
                    self.condition = "maintaining"
            else:
                self.condition = "far"

        else:
            #when things are detected between -5~5
            print(len(self.obstacle_data_range))
            self.condition = "obstacle"

    def maintain_direction(self):
        print(self.DIRECTION)
        if self.DIRECTION == LEFT:
            angle1 = self.angle_distance(65)
            angle2 = self.angle_distance(83)
        elif self.DIRECTION == RIGHT:
            angle1 = self.angle_distance(-65)
            angle2 = self.angle_distance(-70)
            print(angle1)
            print(angle2)
        if angle2 is not None and angle1 is not None:
            try:
                theta = acos(angle2 / angle1)
                print(theta)
                thetad = theta * 180 / pi
                DEFAULT_THETA = 0.35
                if 0 < thetad < 20:
                    print("세타가 작습니다")
                    if self.DIRECTION == LEFT:
                        self.angle = theta - DEFAULT_THETA
                    elif self.DIRECTION == RIGHT:
                        self.angle = -(theta - DEFAULT_THETA)
                        # if angle1 and angle2 == None:
                        #     self.angle = theta - DEFAULT_THETA
                        #     self.speed = self.default_speed
                    self.speed = self.default_speed

                elif thetad > 20:
                    print("세타가 큽니다.")
                    if self.DIRECTION == LEFT:
                        self.angle = -(theta - DEFAULT_THETA)
                    elif self.DIRECTION == RIGHT:
                        self.angle = theta - DEFAULT_THETA
                    self.speed = self.default_speed
            except ValueError:
                pass
        elif angle2 is None and angle1 is None:
            print("angle None, 직진 유지")
            print(self.DIRECTION)
            if self.DIRECTION == LEFT:
                self.angle = 0
            elif self.DIRECTION == RIGHT:
                self.angle = -0.1
                self.speed = self.default_speed
            self.speed = self.default_speed
        else:
            pass
    
    def angle_distance(self, degree):
        for i, data in enumerate(self.msg.ranges):
            if (
                degree < self.degrees[i] < degree + 1
                and 0 < self.msg.ranges[i] < 0.65
            ):
                real_data = self.msg.ranges[i]
                return real_data

    def move_control(self):
        print(self.condition)
        if self.condition == "forward":
            self.speed = self.default_speed
            self.angle = 0
        elif self.condition == "close":
            print("벽과의 거리가 60cm보다 작습니다.")
            if self.DIRECTION == LEFT:
                self.angle = -self.default_angle / (min(self.distance) * 40)
            elif self.DIRECTION == RIGHT:
                self.angle = self.default_angle / (min(self.distance) * 40) * 0.7
        elif self.condition == "far":
            print("벽과의 거리가 60cm보다 큽니다")
            if self.DIRECTION == LEFT:
                self.angle = (self.default_angle) * 0.7
            elif self.DIRECTION == RIGHT:
                self.angle = -(self.default_angle) * 0.7
        elif self.condition == "maintaining":
            self.maintain_direction()
        elif self.condition == "obstacle":
            self.obstacle_motion()
        self.obstacle_data_idx = []
        self.obstacle_data_range = []
        self.distance = []

    def obstacle_motion(self):
        print("obstacle_motion")
        angle_incre = len(self.obstacle_data_idx) / 10 * pi / 180
        obstacle_end_point = self.obstacle_data_idx[-1]
        blank_space = len(self.obstacle_data_idx) - obstacle_end_point
        turn_angle = angle_incre * blank_space / 10
        # print(len(self.obstacle_data_idx))
        if self.DIRECTION == LEFT:
            self.angle = turn_angle
        elif self.DIRECTION == RIGHT:
            self.angle = -turn_angle
        self.speed = 0.06

    def main(self):
        if self.is_scan:
            self.LiDAR_scan()
            self.judge_distance()
            self.move_control()
            self.cmd_vel_msg.linear.x = self.speed
            self.cmd_vel_msg.angular.z = self.angle
            self.pub.publish(self.cmd_vel_msg)
            self.is_scan = False


# Check if this is the main module that is being run
if __name__ == "__main__":
    # Create an instance of the Limo_obstacle_avoidence class
    limo_wall_following = Limo_wall_following(RIGHT)
    try:
        # Start a loop that will continue until ROS is shutdown
        while not rospy.is_shutdown():
            # Call the main method of the Limo_obstacle_avoidence class
            limo_wall_following.main()
    # Catch the ROSInterruptException to ensure a clean exit
    except rospy.ROSInterruptException:
        pass
