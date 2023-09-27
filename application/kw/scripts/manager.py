#! /usr/bin/env python3

import rospy
from enum import Enum

from std_msgs.msg import String
from kw_msgs.srv import Button, ButtonResponse
from kw_msgs.msg import CarrotAction, CarrotGoal, CarrotResult
import actionlib

class RobotMode(Enum):
    Default = 0
    SavingOne = 1
    SavingTwo = 2
    SavingThree =3
    FollowingOne = 4
    FollowingTwo = 5
    FollowingThree = 6
    Stop = 7
    Sequence = 8

class SequenceMode(Enum):
    Default = 0
    PathOne = 1
    PathTwo = 2
    PathThree = 3
    Resume = 4

class Manager:
    def __init__(self):
        self.robot_status = RobotMode(0)
        self.request_status = RobotMode(0)
        self.sequence_status = SequenceMode(0)
        self.pause_status = SequenceMode(0)
        self.gui_service = rospy.Service('/kw/button', Button, self.setStatus)
        self.gui_pub = rospy.Publisher("/kw/status", String, queue_size=10)
        self.carrot_client = actionlib.SimpleActionClient('/kw/carrot', CarrotAction)
        self.carrot_client.wait_for_server()
        self.action_goal = CarrotGoal()

    def setStatus(self, request):
        if request.btn == "follow_path_1":
            self.request_status = RobotMode.FollowingOne
        elif request.btn == "follow_path_2":
            self.request_status = RobotMode.FollowingTwo
        elif request.btn == "follow_path_3":
            self.request_status = RobotMode.FollowingThree
        elif request.btn == "save_path_1":
            self.request_status = RobotMode.SavingOne
        elif request.btn == "save_path_2":
            self.request_status = RobotMode.SavingTwo
        elif request.btn == "save_path_3":
            self.request_status = RobotMode.SavingThree
        elif request.btn == "aio":
            self.request_status = RobotMode.Sequence
        else:
            self.request_status = RobotMode.Stop
        return ButtonResponse(True)
    # 현재 status checking 하며 publish

    def publishStatus(self):
        msg = String()
        if self.robot_status == RobotMode.SavingOne:
            msg.data = "save path one"
        elif self.robot_status == RobotMode.SavingTwo:
            msg.data = "save path two"
        elif self.robot_status == RobotMode.SavingThree:
            msg.data = "save path three"
        elif self.robot_status == RobotMode.FollowingOne:
            msg.data = "follow path one"
        elif self.robot_status == RobotMode.FollowingTwo:
            msg.data = "follow path two"
        elif self.robot_status == RobotMode.FollowingThree:
            msg.data = "follow path three"
        elif self.robot_status == RobotMode.Sequence:
            msg.data = "follow sequence"
        else:
            msg.data = "waiting..."

        self.gui_pub.publish(msg)
    # follow나 saving 상태에는 실행
    # stop에는 아무것도 하지 않기

    def setMotion(self):
        if self.robot_status == RobotMode.Default:
            if self.request_status == RobotMode.Stop or self.request_status == RobotMode.Default:
                pass
            elif self.request_status == RobotMode.SavingOne or self.request_status == RobotMode.SavingTwo or self.request_status == RobotMode.SavingThree:
                self.save()
            elif self.request_status == RobotMode.FollowingOne or self.request_status == RobotMode.FollowingTwo or self.request_status == RobotMode.FollowingThree:
                self.follow()
            elif self.request_status == RobotMode.Sequence:
                if self.pause_status == SequenceMode.PathOne or self.pause_status == SequenceMode.PathTwo or self.pause_status == SequenceMode.PathThree:
                    self.sequence_status = SequenceMode.Resume
                    self.sequence()
                else:
                    self.sequence()
            else:
                pass
        elif self.robot_status == RobotMode.Sequence:
            if self.request_status == RobotMode.Stop:
                self.pause_status = self.sequence_status
                # pause_status = stop되기 전의 sequence_status
                self.stop()
                self.sequence_status = SequenceMode.Default
            else:
                self.sequence()
        else:
            if self.request_status == RobotMode.Stop:
                self.stop()
            else:
                pass
        self.request_status = RobotMode.Default
        
    def save(self):
        self.action_goal.func = 0
        # saving
        if self.request_status == RobotMode.SavingOne:
            self.action_goal.path =1
            self.robot_status = RobotMode.SavingOne
        elif self.request_status == RobotMode.SavingTwo:
            self.action_goal.path =2
            self.robot_status = RobotMode.SavingTwo
        else:
            self.action_goal.path=3
            self.robot_status = RobotMode.SavingThree

        self.carrot_client.send_goal(self.action_goal)

    def follow(self):
        self.action_goal.func = 1
        if self.request_status == RobotMode.FollowingOne:
            self.action_goal.path =1
            self.robot_status = RobotMode.FollowingOne
            # self.sequence_status = SequenceMode.PathOne
        elif self.request_status == RobotMode.FollowingTwo:
            self.action_goal.path =2
            self.robot_status = RobotMode.FollowingTwo
            # self.sequence_status = SequenceMode.PathTwo
        else:
            self.action_goal.path=3
            self.robot_status = RobotMode.FollowingThree
            # self.sequence_status = SequenceMode.PathThree

        self.carrot_client.send_goal(self.action_goal)

    def sequence(self):
        if str(self.carrot_client.get_state()) != '1':
            self.action_goal.func = 1
            if self.sequence_status == SequenceMode.Default:
                self.robot_status = RobotMode.Sequence
                self.action_goal.path = 1
                self.sequence_status = SequenceMode.PathOne
                self.carrot_client.send_goal(self.action_goal) 
            elif self.sequence_status == SequenceMode.PathOne:
                self.robot_status = RobotMode.Sequence
                self.action_goal.path =2
                self.sequence_status = SequenceMode.PathTwo
                self.carrot_client.send_goal(self.action_goal)
            elif self.sequence_status == SequenceMode.PathTwo:
                self.robot_status = RobotMode.Sequence
                self.action_goal.path =3
                self.sequence_status = SequenceMode.PathThree
                self.carrot_client.send_goal(self.action_goal)
            elif self.sequence_status == SequenceMode.PathThree:
                self.sequence_status = SequenceMode.Default
                self.pause_status = SequenceMode.Default
            elif self.sequence_status == SequenceMode.Resume:
                if self.pause_status == SequenceMode.PathOne:
                    self.sequence_status = SequenceMode.Default
                    self.sequence()
                elif self.pause_status == SequenceMode.PathTwo:
                    self.sequence_status = SequenceMode.PathOne
                    self.sequence()
                elif self.pause_status == SequenceMode.PathThree:
                    self.sequence_status = SequenceMode.PathTwo
                    self.sequence()
                else:
                    self.sequence_status = SequenceMode.Default
                    self.pause_status = SequenceMode.Default
        else:
            pass
    
    def stop(self):
        self.carrot_client.cancel_all_goals()

    ## 끝났거나 취소 요청시 default로 둠

    def getActionStatus(self):
        if str(self.carrot_client.get_state()) == '1': #str(self.carrot_client.get_state()) == 'ACTIVE': ## 3은 나중에 처리해야 할 것 같음 요기는 action이 돌아가고 있을 때임
            pass
        elif self.robot_status == RobotMode.Sequence:
            if self.sequence_status == SequenceMode.Default:
                self.robot_status = RobotMode.Default
            else:
                pass
        else:
            self.robot_status = RobotMode.Default

def main():

    try:
        rospy.init_node('manager')
        robo = Manager()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # wait_for_state를 쓰지 않고 이렇게 ~
            robo.getActionStatus()
            robo.publishStatus()
            robo.setMotion()
            rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    
if __name__ == '__main__':
    main()