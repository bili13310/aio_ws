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
        self.default_speed = 0.30
        self.default_angle = 0.2
        self.scan_dist = 0.75
        self.offset = 0.35

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
                print(len(self.obstacle_data_range))

    def judge_distance(self):
        if len(self.obstacle_data_range) == 0:
            self.condition = "forward"
            # self.speed = self.default_speed
            if len(self.distance) > 0:
                if min(self.distance) < self.scan_dist - 0.4:
                    self.condition = "close"

                elif (self.scan_dist - self.offset <= max(self.distance) <= self.scan_dist+self.offset):
                    self.condition = "maintaining"
            else:
                self.condition = "far"

        else:
            # when things are detected between -5~5
            self.condition = "obstacle"

    def maintain_direction(self):
        if self.DIRECTION == LEFT:
            angle1 = self.angle_distance(70)
            angle2 = self.angle_distance(80)
        elif self.DIRECTION == RIGHT:
            angle1 = self.angle_distance(-70)
            angle2 = self.angle_distance(-80)
        if angle2 is not None and angle1 is not None:
            try:
                theta = acos(angle2 / angle1)
                thetad = theta * 180 / pi
                # print(angle1)
                # print(angle2)
                # print(theta)
                # print(thetad)
                DEFAULT_THETA = 0.28
                if 0 < thetad < 20:
                    print("세타가 작습니다")
                    if self.DIRECTION == LEFT:
                        self.angle = theta - DEFAULT_THETA
                    elif self.DIRECTION == RIGHT:
                        self.angle = -(theta - DEFAULT_THETA)
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
        else:
            pass

    def angle_distance(self, degree):
        for i, data in enumerate(self.msg.ranges):
            if (
                degree < self.degrees[i] < degree + 1
                and 0 < self.msg.ranges[i] < 0.9
            ):
                real_data = self.msg.ranges[i]
                return real_data

    def move_control(self):
        if self.condition == "forward":
            self.speed = self.default_speed
            self.angle = 0
        elif self.condition == "close":
            print("벽과의 거리가 60cm보다 작습니다.")
            if self.DIRECTION == LEFT:
                self.angle = -self.default_angle / (min(self.distance) * 10)
            elif self.DIRECTION == RIGHT:
                self.angle = self.default_angle / (min(self.distance) * 10)
        elif self.condition == "far":
            print("벽과의 거리가 60cm보다 큽니다")
            if self.DIRECTION == LEFT:
                self.angle = (self.default_angle) * 2
            elif self.DIRECTION == RIGHT:
                self.angle = -(self.default_angle) * 2
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
        turn_angle = angle_incre * blank_space / 8
        # print(turn_angle)
        if self.DIRECTION == LEFT:
            self.angle = turn_angle
        elif self.DIRECTION == RIGHT:
            self.angle = -turn_angle
        self.speed = 0.10

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