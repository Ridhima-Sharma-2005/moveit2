#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from math import pi

class MyRobot:
    def __init__(self, Group_Name):
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('node_set_redefined_pose', anonymous=True)

        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._planning_group = Group_Name
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        
        # Increase planning and execution timeouts
        self._group.set_planning_time(30)
        self._group.set_num_planning_attempts(10)
        self._group.set_goal_tolerance(0.1)  # Position tolerance in meters
        self._group.set_goal_orientation_tolerance(0.1) 
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo('\033[95m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[95m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[95m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[95m' + " >>> MyRobot initialization is done." + '\033[0m')

    def set_pose(self, arg_pose_name):
        rospy.loginfo('\033[32m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan_success, plan, planning_time, error_code = self._group.plan()
        if plan_success:
            goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
            goal.trajectory = plan
            self._exectute_trajectory_client.send_goal(goal)
            self._exectute_trajectory_client.wait_for_result()
            rospy.loginfo('\033[32m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
        else:
            rospy.logwarn('\033[31m' + "Planning failed. Error code: {}".format(error_code) + '\033[0m')
            rospy.logwarn('\033[31m' + "Planning time: {}".format(planning_time) + '\033[0m')

    def set_pose1(self, pose):
        rospy.loginfo('\033[32m' + "Going to Pose: {}".format(pose) + '\033[0m')
        self._group.set_start_state_to_current_state()
        self._group.set_pose_target(pose, "link_5")
        
        plan_success, plan, planning_time, error_code = self._group.plan()
        
        if plan_success:
            rospy.loginfo('\033[32m' + "Planning successful. Executing the plan." + '\033[0m')
            goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
            goal.trajectory = plan
            self._exectute_trajectory_client.send_goal(goal)
            self._exectute_trajectory_client.wait_for_result()
            rospy.loginfo('\033[32m' + "Now at Pose: {}".format(pose) + '\033[0m')
        else:
            rospy.logwarn('\033[31m' + "Planning failed. Error code: {}".format(error_code) + '\033[0m')
            rospy.logwarn('\033[31m' + "Planning time: {}".format(planning_time) + '\033[0m')
    
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[95m' + "Object of class MyRobot Deleted." + '\033[0m')

def compute_pick_pose(object_position):
    pick_pose = geometry_msgs.msg.Pose()
    pick_pose.position.x = object_position.x
    pick_pose.position.y = object_position.y
    pick_pose.position.z = object_position.z
    pick_pose.orientation.w = 1.0
    pick_pose.orientation.x = -1.5
    pick_pose.orientation.y = -1.5
    pick_pose.orientation.z = -1.5
    return pick_pose

def main():
    arm = MyRobot("arm_group")
    hand = MyRobot("hand")
    object_position = geometry_msgs.msg.Point(0.03,-0.37,0.8)
    pick_pose = compute_pick_pose(object_position)

    while not rospy.is_shutdown():
        arm.set_pose("zero_pose")
        rospy.sleep(2)
        
        arm.set_pose1(pick_pose)
        rospy.sleep(2)
        
        hand.set_pose("hand_opem")
        rospy.sleep(1)
        
        arm.set_pose("pick_object_pose")
        rospy.sleep(2)
        hand.set_pose("hand_closed")
        rospy.sleep(1)
        
        arm.set_pose("lift_object_pose")
        rospy.sleep(2)
        
        arm.set_pose("place_object_opposite_pose")
        rospy.sleep(2)
        
        hand.set_pose("hand_opem")
        rospy.sleep(1)
        
        arm.set_pose("opposite_pose")
        rospy.sleep(2)
        
        hand.set_pose("hand_closed")
        rospy.sleep(1)

    del arm
    del hand

if __name__ == '__main__':
    main()

