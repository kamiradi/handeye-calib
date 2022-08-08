
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

class Trajectory(object):

    def __init__(self):
        rospy.init_node('calibration_trajectory', anonymous=True)
        # rospy.wait_for_message('move_group/status', GoalStatusArray)
        self.commander = MoveGroupCommander('panda_arm')
        self.commander.set_max_velocity_scaling_factor(0.05)
        self.commander.set_max_acceleration_scaling_factor(0.05) 
        self.commander.set_planning_time(10) 
        self.robot = RobotCommander() 
        self.buf = tf2_ros.Buffer() 
        self.tf_listener = tf2_ros.TransformListener(self.buf) 
        self.calibPublisher = rospy.Publisher('calib_data', calib, queue_size=20) 
        self.solvePublisher = rospy.Publisher('shall_solve', Bool)
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
        camera_target_trans = self.buf.lookup_transform( 'tag_1', 'camera_link', rospy.Time(0), rospy.Duration(10))
        world_gripper_trans = self.buf.lookup_transform( 'panda_K', 'panda_link0', rospy.Time(0), rospy.Duration(10))

        camera_target_pose = PoseStamped()
        camera_target_pose.header.stamp = rospy.Time.now()
        camera_target_pose.pose.position.x = camera_target_trans.transform.translation.x
        camera_target_pose.pose.position.y = camera_target_trans.transform.translation.y
        camera_target_pose.pose.position.z = camera_target_trans.transform.translation.z
        camera_target_pose.pose.orientation.x = camera_target_trans.transform.rotation.x
        camera_target_pose.pose.orientation.y = camera_target_trans.transform.rotation.y
        camera_target_pose.pose.orientation.z = camera_target_trans.transform.rotation.z
        camera_target_pose.pose.orientation.w = camera_target_trans.transform.rotation.w

        world_gripper_pose = PoseStamped()
        world_gripper_pose.header.stamp = rospy.Time.now()
        world_gripper_pose.pose.position.x = world_gripper_trans.transform.translation.x
        world_gripper_pose.pose.position.y = world_gripper_trans.transform.translation.y
        world_gripper_pose.pose.position.z = world_gripper_trans.transform.translation.z
        world_gripper_pose.pose.orientation.x = world_gripper_trans.transform.rotation.x
        world_gripper_pose.pose.orientation.y = world_gripper_trans.transform.rotation.y
        world_gripper_pose.pose.orientation.z = world_gripper_trans.transform.rotation.z
        world_gripper_pose.pose.orientation.w = world_gripper_trans.transform.rotation.w

        calib_msg = calib()
        calib_msg.camera_target = camera_target_pose
        calib_msg.base_gripper = world_gripper_pose
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
    
