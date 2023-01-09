
import rospy
from moveit_commander import MoveGroupCommander, RobotCommander
from actionlib_msgs.msg import GoalStatusArray
import copy
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
# from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal, HomingGoal, HomingAction, HomingActionGoal
import actionlib
import moveit_msgs.msg
from std_msgs.msg import Bool
from math import cos
import tf
import tf2_ros
import pdb
from handeye_calib.msg import calib
import sys

class Trajectory(object):

    def __init__(self):
        rospy.init_node('calibration_trajectory', anonymous=True)
        # rospy.wait_for_message('move_group/status', GoalStatusArray)
        self.commander = MoveGroupCommander('panda_realsense_arm')
        self.commander.set_max_velocity_scaling_factor(0.05)
        self.commander.set_max_acceleration_scaling_factor(0.05) 
        self.commander.set_planning_time(10) 
        self.robot = RobotCommander() 
        self.buf = tf2_ros.Buffer() 
        self.tf_listener = tf2_ros.TransformListener(self.buf) 
        self.calibPublisher = rospy.Publisher('calib_data', calib, queue_size=20) 
        self.solvePublisher = rospy.Publisher('shall_solve', Bool, queue_size=1)
        # self.gripperPublisher = rospy.Publisher('base_gripper', calib, queue_size=20)
        r = rospy.Rate(10)
        rospy.sleep(5)
        print("Trajectory node ready ..")
        # print("Ref frame: " + self.commander.get_planning_frame())
        # print("End effector: " + self.commander.get_end_effector_link()) 
        # print("initialised trajectory")

    def move(self):
        try:
            initial_pose = copy.deepcopy(self.commander.get_current_pose().pose)
            print(initial_pose)

            # ideal start pose
            # - Translation: [0.319, -0.107, 0.596]
            # - Rotation: in Quaternion [0.922, -0.380, 0.072, -0.015]

            # use this and manually set orientation (need to find a better way)
            # move_to_pos(0.303, -0.105, 0.586)
            print("preparing to move")
            self.calibration_path()
            print("trajectory executed")
            shall_solve = Bool()
            shall_solve.data = True
            self.solvePublisher.publish(shall_solve)
            self.move_to_pos(initial_pose.position.x, initial_pose.position.y, initial_pose.position.z)
            
        except rospy.ROSInterruptException:
            pass  

    def move_to_pos(self, x, y, z):
        start_pose = self.commander.get_current_pose().pose 
        new_pose = copy.deepcopy(start_pose)
        new_pose.position.x = x
        new_pose.position.y = y
        new_pose.position.z = z
        self.commander.set_pose_target(new_pose)
        self.commander.plan()
        self.commander.go(wait=True)
        self.commander.stop() 
        rospy.sleep(5)

        # extract tag -> camera transformation and gripper -> world
        # transformation
        X_TagCamera = self.buf.lookup_transform( 'tag_1', 'camera_link', rospy.Time(0), rospy.Duration(10))
        X_GripperBase = self.buf.lookup_transform( 'panda_K', 'panda_link0', rospy.Time(0), rospy.Duration(10))
        # X_TagCamera = self.buf.lookup_transform('camera_link', 'tag_1', rospy.Time(0), rospy.Duration(10))
        # X_GripperBase = self.buf.lookup_transform('panda_link0', 'panda_K', rospy.Time(0), rospy.Duration(10))

        tag_camera_pose = PoseStamped()
        tag_camera_pose.header.stamp = rospy.Time.now()
        tag_camera_pose.pose.position.x = X_TagCamera.transform.translation.x
        tag_camera_pose.pose.position.y = X_TagCamera.transform.translation.y
        tag_camera_pose.pose.position.z = X_TagCamera.transform.translation.z
        tag_camera_pose.pose.orientation.x = X_TagCamera.transform.rotation.x
        tag_camera_pose.pose.orientation.y = X_TagCamera.transform.rotation.y
        tag_camera_pose.pose.orientation.z = X_TagCamera.transform.rotation.z
        tag_camera_pose.pose.orientation.w = X_TagCamera.transform.rotation.w

        gripper_base_pose = PoseStamped()
        gripper_base_pose.header.stamp = rospy.Time.now()
        gripper_base_pose.pose.position.x = X_GripperBase.transform.translation.x
        gripper_base_pose.pose.position.y = X_GripperBase.transform.translation.y
        gripper_base_pose.pose.position.z = X_GripperBase.transform.translation.z
        gripper_base_pose.pose.orientation.x = X_GripperBase.transform.rotation.x
        gripper_base_pose.pose.orientation.y = X_GripperBase.transform.rotation.y
        gripper_base_pose.pose.orientation.z = X_GripperBase.transform.rotation.z
        gripper_base_pose.pose.orientation.w = X_GripperBase.transform.rotation.w

        calib_msg = calib()
        calib_msg.tag_camera = tag_camera_pose
        calib_msg.gripper_base = gripper_base_pose
        self.calibPublisher.publish(calib_msg)
        # self.gripperPublisher.publish(world_gripper_pose)


        return new_pose

    def change_orientation(self, x, y, z, w):
        start_pose = self.commander.get_current_pose().pose 
        new_pose = copy.deepcopy(start_pose)
        new_pose.orientation.x = x
        new_pose.orientation.y = y
        new_pose.orientation.z = z
        new_pose.orientation.w = w
        self.commander.set_pose_target(new_pose)
        self.commander.plan()
        self.commander.go(wait=True)
        self.commander.stop()
        return new_pose

    def calibration_path(self):
        start_pose = self.commander.get_current_pose().pose
        pos_x = start_pose.position.x
        pos_y = start_pose.position.y
        pos_z = start_pose.position.z
        # ori_x = start_pose.orientation.x
        # ori_y = start_pose.orientation.y
        # ori_z = start_pose.orientation.z
        # ori_w = start_pose.orientation.w

        for i in range(5):
            pos_x += 0.03
            self.move_to_pos(pos_x, pos_y, pos_z)
            rospy.sleep(5)

        for i in range(3):
            pos_y -= 0.05
            self.move_to_pos(pos_x, pos_y, pos_z)
            rospy.sleep(5)

        pos_z -= 0.05
        self.move_to_pos(pos_x, pos_y, pos_z)
        rospy.sleep(5)

        for i in range(3):
            pos_x -= 0.02
            self.move_to_pos(pos_x, pos_y, pos_z)
            rospy.sleep(5)

        pos_z -= 0.05
        self.move_to_pos(pos_x, pos_y, pos_z)
        rospy.sleep(5)

        pos_x += 0.03
        pos_y -= 0.03
        self.move_to_pos(pos_x, pos_y, pos_z)
        rospy.sleep(5)

        for i in range(2):
            pos_y += 0.1
            self.move_to_pos(pos_x, pos_y, pos_z)
            rospy.sleep(5)

        pos_z -= 0.02
        self.move_to_pos(pos_x, pos_y, pos_z)
        rospy.sleep(5)

        for i in range(3):
            pos_y -= 0.04
            self.move_to_pos(pos_x, pos_y, pos_z)
            rospy.sleep(5)


def main():
    traj = Trajectory() 
    traj.move()

if __name__ == '__main__':
    
    main()
    
