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
        rospy.Subscriber("limo/scan", LaserScan, self.laser_callback)
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
        self.scan_dist = 0.75

        self.msg = None

    def laser_callback(self, msg):
        self.msg = msg
        self.is_scan = True
        # print("callback")

    def LiDAR_scan(self):
        # print("LiDAR_scan")
        if self.lidar_flag == False:
            # self.msg.angle_min = -2.094399929046631
            # len(self.msg.ranges) = 720
            # check the components of ranges from the photo i captured
            self.degrees = [
                (self.msg.angle_min + (i * self.msg.angle_increment)) * 180 / pi
                for i, data in enumerate(self.msg.ranges)
            ]
            # check the components of degrees from the photo i captured
            self.lidar_flag = True

        # 원하는 각도 내에서 원하는 거리값 이내 값 뽑기
        for i, data in enumerate(self.msg.ranges):
            if -70 < self.degrees[i] < 70 and 0 < self.msg.ranges[i] <= self.scan_dist:
                self.distance.append(data)

        # 전방 장애물 감지 각도
        for i, data in enumerate(self.msg.ranges):
            if 2 < self.degrees[i] < 4 and 0 < self.msg.ranges[i] < self.scan_dist:
                self.obstacle_data_idx.append(i)
                self.obstacle_data_range.append(data)
                print(self.obstacle_data_idx)

    def angle_distance(self, degree):
        for i, data in enumerate(self.msg.ranges):
            if (
                degree < self.degrees[i] < degree + 1
                and 0 < self.msg.ranges[i] < 0.9
            ):
                real_data = self.msg.ranges[i]
                return real_data

    def main(self):
        if self.is_scan:
            self.LiDAR_scan()
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