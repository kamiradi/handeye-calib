import rospy
from moveit_commander import MoveGroupCommander, RobotCommander
from actionlib_msgs.msg import GoalStatusArray
import copy
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose
from std_msgs.msg import Bool
import actionlib 
import moveit_msgs.msg
from math import cos
import tf
import tf2_ros
import tf_conversions.posemath as pm
from handeye_calib.msg import calib
import sys
sys.path.insert(0, "/home/panda/.local/lib/python2.7/site-packages")
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from pyquaternion import Quaternion
from threading import Lock
import PyKDL

class Calibration():
	def __init__(self):
		rospy.init_node('calibration_solver', anonymous=True)
		rospy.Subscriber('calib_data', calib, self._accumulate)
		rospy.Subscriber('shall_solve', Bool, self._solve)
		self.solution = tf2_ros.StaticTransformBroadcaster()
		self.debug_frames = tf2_ros.StaticTransformBroadcaster()
		# self.R_CamTag = []
		# self.p_CamTag_Cam = []
		# self.R_BaseGripper = []
		# self.p_BaseGripper_Base = []
		self.R_TagCam = []
		self.p_TagCam_Tag = []
		self.R_GripperBase = []
		self.p_GripperBase_Gripper = []
		self.data_points = 0
		self.append_lock = Lock()
		print("Calibration node ready ..")
	

	def _solve(self, data):
		# R_baseGripper = np.array(self.R_BaseGripper)
		# R_CamTag = np.array(self.R_CamTag)
		# p_BaseGripper_Base = np.array(self.p_BaseGripper_Base)
		# p_CamTag_Cam = np.array(self.p_CamTag_Cam)
		# R_baseGripper = self.R_BaseGripper
		# R_CamTag = self.R_CamTag
		# p_BaseGripper_Base = self.p_BaseGripper_Base
		# p_CamTag_Cam = self.p_CamTag_Cam
		R_GripperBase = np.array(self.R_GripperBase)
		p_GripperBase_Gripper = np.array(self.p_GripperBase_Gripper)
		R_TagCam = np.array(self.R_TagCam)
		p_TagCam_Tag = np.array(self.p_TagCam_Tag)

		print("size of rotations and translations: {}, {}, {}, {}".format(R_GripperBase.shape, R_TagCam.shape, p_GripperBase_Gripper.shape, p_TagCam_Tag.shape))
		print("solving ...")
		# R_CamGrip, p_CamGrip_Cam = cv2.calibrateHandEye(R_baseGripper, p_BaseGripper_Base, R_CamTag, p_CamTag_Cam)
		# R_CamGrip, p_CamGrip_Cam = cv2.calibrateHandEye(R_gripper2base = R_GripperBase, t_gripper2base = p_GripperBase_Gripper, R_target2cam = R_TagCam, t_target2cam = p_TagCam_Tag, method=cv2.CALIB_HAND_EYE_DANIILIDIS)
		R_CamGrip, p_CamGrip_Cam = cv2.calibrateHandEye(R_GripperBase, p_GripperBase_Gripper, R_TagCam, p_TagCam_Tag)
		print("Solved ...")
		print("R: {}".format(R_CamGrip))
		print("t: {}".format(p_CamGrip_Cam))
		# solution_quat = R.as_quat(R.from_dcm(R_GripCam))
		solution_quat = Quaternion(matrix=R_CamGrip)
		# temp = np.matmul(np.transpose(R_CamGrip), np.array(p_CamGrip_Cam))
		# p_GripCam_Grip = -1*(temp)
		
		# extract kdl frame to perform transformtations
		X_CamGrip_pose = Pose()
		X_CamGrip_pose.position.x = p_CamGrip_Cam[0]/100
		X_CamGrip_pose.position.y = p_CamGrip_Cam[1]/100
		X_CamGrip_pose.position.z = p_CamGrip_Cam[2]/100

		X_CamGrip_pose.orientation.w = solution_quat[0]
		X_CamGrip_pose.orientation.x = solution_quat[1]
		X_CamGrip_pose.orientation.y = solution_quat[2]
		X_CamGrip_pose.orientation.z = solution_quat[3]
		
		X_CamGrip_frame = pm.fromMsg(X_CamGrip_pose)
		X_GripCam_frame = X_CamGrip_frame.Inverse()
		X_GripCam_pose = pm.toMsg(X_GripCam_frame)
		
		transform = TransformStamped()
		transform.header.stamp = rospy.Time.now()
		transform.header.frame_id = 'panda_K'
		transform.child_frame_id = 'camera_link'
		transform.transform.translation.x = X_GripCam_pose.position.x
		transform.transform.translation.y = X_GripCam_pose.position.y
		transform.transform.translation.z = X_GripCam_pose.position.z

		transform.transform.rotation.w = X_GripCam_pose.orientation.w
		transform.transform.rotation.x = X_GripCam_pose.orientation.x
		transform.transform.rotation.y = X_GripCam_pose.orientation.y
		transform.transform.rotation.z = X_GripCam_pose.orientation.z

		self.solution.sendTransform(transform)

	def _accumulate(self, calib_data):
		tag_camera = calib_data.tag_camera
		gripper_base = calib_data.gripper_base
		# print("camera_target transform: {}".format(calib_data.camera_target))
		# print("base to gripper transform: {}".format(calib_data.base_gripper))
		# sys.exit()

		# extract quaternions and translations
		# This block adds all the necessary translation and rotation quats to lists as
		# required by the opencv calibrate hand eye
		cam_quat = []
		cam_tra = []
		cam_quat.append(tag_camera.pose.orientation.w)
		cam_quat.append(tag_camera.pose.orientation.x)
		cam_quat.append(tag_camera.pose.orientation.y)
		cam_quat.append(tag_camera.pose.orientation.z)
		cam_tra.append(tag_camera.pose.position.x)
		cam_tra.append(tag_camera.pose.position.y)
		cam_tra.append(tag_camera.pose.position.z)

		# retrieve kdl frame to perform transformations
		# primarily for debugging
		# also to experiment with the kdl interface to perform posemath
		X_tag_cam_frame = pm.fromMsg(tag_camera.pose)
		X_cam_tag_frame = X_tag_cam_frame.Inverse()
		X_cam_tag_pose = pm.toMsg(X_cam_tag_frame)

		# send debuggng transformations for rviz to visualise
		cam_transform = TransformStamped()
		cam_transform.header.stamp = rospy.Time.now()
		cam_transform.header.frame_id = 'camera_link' 
		cam_transform.child_frame_id = 'cam_frame_{}'.format(self.data_points)
		cam_transform.transform.translation.x = X_cam_tag_pose.position.x
		cam_transform.transform.translation.y = X_cam_tag_pose.position.y
		cam_transform.transform.translation.z = X_cam_tag_pose.position.z

		cam_transform.transform.rotation.w = X_cam_tag_pose.orientation.w
		cam_transform.transform.rotation.x = X_cam_tag_pose.orientation.x
		cam_transform.transform.rotation.y = X_cam_tag_pose.orientation.y
		cam_transform.transform.rotation.z = X_cam_tag_pose.orientation.z


		# extract quaternions and translations
		# This block adds all the necessary translation and rotation quats to lists as
		# required by the opencv calibrate hand eye
		grip_quat = []
		grip_tra = []
		grip_quat.append(gripper_base.pose.orientation.w)
		grip_quat.append(gripper_base.pose.orientation.x)
		grip_quat.append(gripper_base.pose.orientation.y)
		grip_quat.append(gripper_base.pose.orientation.z)
		grip_tra.append(gripper_base.pose.position.x)
		grip_tra.append(gripper_base.pose.position.y)
		grip_tra.append(gripper_base.pose.position.z)

		# retrieve kdl frame to perform transformations
		# primarily for debugging
		# also to experiment with the kdl interface to perform posemath
		X_grip_base_frame = pm.fromMsg(gripper_base.pose)
		X_base_grip_frame = X_grip_base_frame.Inverse()
		X_base_grip_pose = pm.toMsg(X_base_grip_frame)

		# send debuggng transformations for rviz to visualise
		grip_transform = TransformStamped()
		grip_transform.header.stamp = rospy.Time.now()
		grip_transform.header.frame_id = 'panda_link0' 
		grip_transform.child_frame_id = 'grip_frame_{}'.format(self.data_points)
		grip_transform.transform.translation.x = X_base_grip_pose.position.x
		grip_transform.transform.translation.y = X_base_grip_pose.position.y
		grip_transform.transform.translation.z = X_base_grip_pose.position.z

		grip_transform.transform.rotation.w = X_base_grip_pose.orientation.w
		grip_transform.transform.rotation.x = X_base_grip_pose.orientation.x
		grip_transform.transform.rotation.y = X_base_grip_pose.orientation.y
		grip_transform.transform.rotation.z = X_base_grip_pose.orientation.z
		# self.R_CamTag.append(Quaternion(cam_quat))
		# self.R_BaseGripper.append(Quaternion(grip_quat))
		# self.p_CamTag_Cam.append(np.array(cam_tra))
		# self.p_BaseGripper_Base.append(np.array(grip_tra))
		
		# send trajectory waypoints to visualise in rviz: sanity check
		self.debug_frames.sendTransform(cam_transform)
		self.debug_frames.sendTransform(grip_transform)
		
		# changing reference frames to suit calibrateHandEye functionality in opencv
		with self.append_lock:
			self.R_TagCam.append(Quaternion(cam_quat).rotation_matrix)
			self.R_GripperBase.append(Quaternion(grip_quat).rotation_matrix)
			self.p_TagCam_Tag.append(np.array(cam_tra))
			self.p_GripperBase_Gripper.append(np.array(grip_tra))
			self.data_points += 1
			print("Transforms accumulated: {}".format(self.data_points))

	def quaternion_rotation_matrix(self, Q):
		"""
		Covert a quaternion into a full three-dimensional rotation matrix.
	
		Input
		:param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
	
		Output
		:return: A 3x3 element matrix representing the full 3D rotation matrix. 
				This rotation matrix converts a point in the local reference 
				frame to a point in the global reference frame.
		"""
		# Extract the values from Q
		q0 = Q[3]  #w
		q1 = Q[0]  #x
		q2 = Q[1]  #y
		q3 = Q[2]  #z
		# First row of the rotation matrix
		r00 = 2 * (q0 * q0 + q1 * q1) - 1
		r01 = 2 * (q1 * q2 - q0 * q3)
		r02 = 2 * (q1 * q3 + q0 * q2)
		# Second row of the rotation matrix
		r10 = 2 * (q1 * q2 + q0 * q3)
		r11 = 2 * (q0 * q0 + q2 * q2) - 1
		r12 = 2 * (q2 * q3 - q0 * q1)
		# Third row of the rotation matrix
		r20 = 2 * (q1 * q3 - q0 * q2)
		r21 = 2 * (q2 * q3 + q0 * q1)
		r22 = 2 * (q0 * q0 + q3 * q3) - 1
		# 3x3 rotation matrix
		rot_matrix = np.array([[r00, r01, r02],
							[r10, r11, r12],
							[r20, r21, r22]])
		return rot_matrix


def main():
	calibrate = Calibration()
	rospy.spin()

if __name__ == "__main__":
	main()