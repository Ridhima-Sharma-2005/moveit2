#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import shape_msgs.msg
import tf
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from math import pi

class MyRobot:

    def __init__(self, arm_group_name, hand_group_name):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('node_set_redefined_pose', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name)
        self.hand_group = moveit_commander.MoveGroupCommander(hand_group_name)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self.execute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self.execute_trajectory_client.wait_for_server()

        self.planning_frame = self.arm_group.get_planning_frame()
        self.eef_link = self.arm_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        self.arm_group.set_planner_id("RRTConnectkConfigDefault")

        rospy.loginfo('\033[95m' + "Planning Frame: {}".format(self.planning_frame) + '\033[0m')
        rospy.loginfo('\033[95m' + "End Effector Link: {}".format(self.eef_link) + '\033[0m')
        rospy.loginfo('\033[95m' + "Group Names: {}".format(self.group_names) + '\033[0m')
        rospy.loginfo('\033[95m' + " >>> MyRobot initialization is done." + '\033[0m')

    def create_object(self):
        collision_objects = []

        box1 = moveit_commander.CollisionObject()
        box1.id = "table1"
        box1.header.frame_id = "base_link"
        shape1 = shape_msgs.msg.SolidPrimitive()
        shape1.type = shape1.BOX
        shape1.dimensions = [0.2, 0.4, 0.4]
        pose1 = geometry_msgs.msg.Pose()
        pose1.position.x = 0.5
        pose1.position.y = 0
        pose1.position.z = 0.2
        pose1.orientation.w = 1.0
        box1.primitives.append(shape1)
        box1.primitive_poses.append(pose1)
        box1.operation = box1.ADD
        collision_objects.append(box1)

        box2 = moveit_commander.CollisionObject()
        box2.id = "table2"
        box2.header.frame_id = "base_link"
        shape2 = shape_msgs.msg.SolidPrimitive()
        shape2.type = shape2.BOX
        shape2.dimensions = [0.4, 0.2, 0.4]
        pose2 = geometry_msgs.msg.Pose()
        pose2.position.x = 0.0
        pose2.position.y = 0.5
        pose2.position.z = 0.2
        pose2.orientation.w = 1.0
        box2.primitives.append(shape2)
        box2.primitive_poses.append(pose2)
        box2.operation = box2.ADD
        collision_objects.append(box2)

        box3 = moveit_commander.CollisionObject()
        box3.id = "object"
        box3.header.frame_id = "base_link"
        shape3 = shape_msgs.msg.SolidPrimitive()
        shape3.type = shape3.BOX
        shape3.dimensions = [0.02, 0.02, 0.2]
        pose3 = geometry_msgs.msg.Pose()
        pose3.position.x = 0.5
        pose3.position.y = 0
        pose3.position.z = 0.5
        pose3.orientation.w = 1.0
        box3.primitives.append(shape3)
        box3.primitive_poses.append(pose3)
        box3.operation = box3.ADD
        collision_objects.append(box3)

        for obj in collision_objects:
            self.scene.add_object(obj)

    def open_gripper(self):
        grasp_posture = JointTrajectory()
        grasp_posture.joint_names = ["joint_6", "joint_7"]
        point = JointTrajectoryPoint()
        point.positions = [0.03, -0.03]  # Open positions
        point.time_from_start = rospy.Duration(0.5)
        grasp_posture.points.append(point)
        self.hand_group.set_joint_value_target(grasp_posture)
        self.hand_group.go(wait=True)
        self.hand_group.stop()
        self.hand_group.clear_pose_targets()

    def close_gripper(self):
        grasp_posture = JointTrajectory()
        grasp_posture.joint_names = ["joint_6", "joint_7"]
        point = JointTrajectoryPoint()
        point.positions = [0.00, 0.00]  # Closed positions
        point.time_from_start = rospy.Duration(0.5)
        grasp_posture.points.append(point)
        self.hand_group.set_joint_value_target(grasp_posture)
        self.hand_group.go(wait=True)
        self.hand_group.stop()
        self.hand_group.clear_pose_targets()

    def set_grasp(self):
        grasps = []
        grasp = moveit_msgs.msg.Grasp()
        grasp.allowed_touch_objects = ["object"]
        grasp.allowed_planning_time = 10.0
        grasp.tolerance = 0.01
        grasp.grasp_pose.header.frame_id = "base_link"
        orientation = tf.transformations.quaternion_from_euler(-pi / 2, -pi / 4, -pi / 2)
        grasp.grasp_pose.pose.orientation.x = orientation[0]
        grasp.grasp_pose.pose.orientation.y = orientation[1]
        grasp.grasp_pose.pose.orientation.z = orientation[2]
        grasp.grasp_pose.pose.orientation.w = orientation[3]
        grasp.grasp_pose.pose.position.x = 0.4
        grasp.grasp_pose.pose.position.y = 0
        grasp.grasp_pose.pose.position.z = 0.5

        pre_grasp_approach = moveit_msgs.msg.GripperTranslation()
        pre_grasp_approach.direction.header.frame_id = "base_link"
        pre_grasp_approach.direction.vector.x = 1.0
        rospy.loginfo('\033[95m' + "graspdebug")
        pre_grasp_approach.min_distance = 0.095
        pre_grasp_approach.desired_distance = 0.115
        grasp.pre_grasp_approach = pre_grasp_approach

        post_grasp_retreat = moveit_msgs.msg.GripperTranslation()
        post_grasp_retreat.direction.header.frame_id = "base_link"
        post_grasp_retreat.direction.vector.z = 1.0
        post_grasp_retreat.min_distance = 0.1
        post_grasp_retreat.desired_distance = 0.25
        grasp.post_grasp_retreat = post_grasp_retreat

        grasp.pre_grasp_posture = JointTrajectory()
        grasp.pre_grasp_posture.joint_names = ["joint_6", "joint_7"]
        pre_grasp_point = JointTrajectoryPoint()
        pre_grasp_point.positions = [0.03, -0.03]  # Open
        pre_grasp_point.time_from_start = rospy.Duration(0.5)
        grasp.pre_grasp_posture.points.append(pre_grasp_point)

        grasp.grasp_posture = JointTrajectory()
        grasp.grasp_posture.joint_names = ["joint_6", "joint_7"]
        grasp_point = JointTrajectoryPoint()
        grasp_point.positions = [0.00, 0.00]  # Closed
        grasp_point.time_from_start = rospy.Duration(0.5)
        grasp.grasp_posture.points.append(grasp_point)

        grasps.append(grasp)
        self.arm_group.set_support_surface_name("table1")
        rospy.loginfo('\033[95m' + "graspdebug")
        self.arm_group.pick("object", grasps)

    def place(self):
        place_location = []

        place = moveit_msgs.msg.PlaceLocation()
        place.place_pose.header.frame_id = "base_link"
        orientation = tf.transformations.quaternion_from_euler(0, 0, pi / 2)  # A quarter turn about the z-axis
        place.place_pose.pose.orientation.x = orientation[0]
        place.place_pose.pose.orientation.y = orientation[1]
        place.place_pose.pose.orientation.z = orientation[2]
        place.place_pose.pose.orientation.w = orientation[3]
        place.place_pose.pose.position.x = 0
        place.place_pose.pose.position.y = 0.5
        place.place_pose.pose.position.z = 0.5

        pre_place_approach = moveit_msgs.msg.GripperTranslation()
        pre_place_approach.direction.header.frame_id = "base_link"
        pre_place_approach.direction.vector.z = -1.0  # Direction is set as negative z axis
        pre_place_approach.min_distance = 0.095
        pre_place_approach.desired_distance = 0.115
        place.pre_place_approach = pre_place_approach

        post_place_retreat = moveit_msgs.msg.GripperTranslation()
        post_place_retreat.direction.header.frame_id = "base_link"
        post_place_retreat.direction.vector.y = -1.0  # Direction is set as negative y axis
        post_place_retreat.min_distance = 0.1
        post_place_retreat.desired_distance = 0.25
        place.post_place_retreat = post_place_retreat

        place.post_place_posture = JointTrajectory()
        place.post_place_posture.joint_names = ["joint_6", "joint_7"]
        open_point = JointTrajectoryPoint()
        open_point.positions = [0.03, -0.03]  # Open positions
        open_point.time_from_start = rospy.Duration(0.5)
        place.post_place_posture.points.append(open_point)

        place_location.append(place)

        self.arm_group.set_support_surface_name("table2")
        result = self.arm_group.place("object", place_location)

        if result:
            rospy.loginfo("Place operation successful.")
        else:
            rospy.logwarn("Place operation failed.")
            self.arm_group.clear_pose_targets()

if __name__ == '__main__':
    try:
        robot = MyRobot("arm_group", "hand")
        robot.create_object()
        robot.set_grasp()
        rospy.loginfo('\033[95m' + "objcreated")
        rospy.sleep(50)  # Wait for the grasp operation to complete
        #robot.place()
    except rospy.ROSInterruptException:
        pass

