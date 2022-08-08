import rospy
from moveit_commander import MoveGroupCommander, RobotCommander
from actionlib_msgs.msg import GoalStatusArray
import copy
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool
import actionlib 
import moveit_msgs.msg
from math import cos
import tf
import tf2_ros
from handeye_calib.msg import calib
import sys
sys.path.insert(0, "/home/panda/.local/lib/python2.7/site-packages")
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from pyquaternion import Quaternion


class Calibration():
	def __init__(self):
		rospy.init_node('calibration_solver', anonymous=True)
		rospy.Subscriber('calib_data', calib, self._accumulate)
		rospy.Subscriber('shall_solve', Bool, self._solve)
		self.solution = tf2_ros.StaticTransformBroadcaster()
		# self.R_CamTag = []
		# self.p_CamTag_Cam = []
		# self.R_BaseGripper = []
		# self.p_BaseGripper_Base = []
		self.R_TagCam = []
		self.p_TagCam_Tag = []
		self.R_GripperBase = []
		self.p_GripperBase_Gripper = []
		self.data_points = 0
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
		R_GripperBase = self.R_GripperBase
		p_GripperBase_Gripper = self.p_GripperBase_Gripper
		R_TagCam = self.R_TagCam
		p_TagCam_Tag = self.p_TagCam_Tag
		print("solving ...")
		# R_CamGrip, p_CamGrip_Cam = cv2.calibrateHandEye(R_baseGripper, p_BaseGripper_Base, R_CamTag, p_CamTag_Cam)
		R_CamGrip, p_CamGrip_Cam = cv2.calibrateHandEye(R_GripperBase, p_GripperBase_Gripper, R_TagCam, p_TagCam_Tag)
		print("Solved ...")
		print("R: {}".format(R_CamGrip))
		print("t: {}".format(p_CamGrip_Cam))
		# solution_quat = R.as_quat(R.from_dcm(R_GripCam))
		solution_quat = Quaternion(matrix=np.transpose(R_CamGrip))
		temp = np.matmul(np.transpose(R_CamGrip), np.array(p_CamGrip_Cam))
		p_GripCam_Grip = -1*(temp)
		
		transform = TransformStamped()
		transform.header.stamp = rospy.Time.now()
		transform.header.frame_id = 'panda_K'
		transform.child_frame_id = 'camera_link'
		transform.transform.translation.x = p_GripCam_Grip[0]
		transform.transform.translation.y = p_GripCam_Grip[1]
		transform.transform.translation.z = p_GripCam_Grip[2]

		transform.transform.rotation.x = solution_quat[0]
		transform.transform.rotation.y = solution_quat[1]
		transform.transform.rotation.z = solution_quat[2]
		transform.transform.rotation.w = solution_quat[3]

		self.solution.sendTransform(transform)

	def _accumulate(self, calib_data):
		camera_target = calib_data.camera_target
		base_gripper = calib_data.base_gripper

		# extract quaternions and translations
		cam_quat = []
		cam_tra = []
		cam_quat.append(camera_target.pose.orientation.x)
		cam_quat.append(camera_target.pose.orientation.y)
		cam_quat.append(camera_target.pose.orientation.z)
		cam_quat.append(camera_target.pose.orientation.w)
		cam_tra.append(camera_target.pose.position.x)
		cam_tra.append(camera_target.pose.position.y)
		cam_tra.append(camera_target.pose.position.z)

		grip_quat = []
		grip_tra = []
		grip_quat.append(base_gripper.pose.orientation.x)
		grip_quat.append(base_gripper.pose.orientation.y)
		grip_quat.append(base_gripper.pose.orientation.z)
		grip_quat.append(base_gripper.pose.orientation.w)
		grip_tra.append(base_gripper.pose.position.x)
		grip_tra.append(base_gripper.pose.position.y)
		grip_tra.append(base_gripper.pose.position.z)

		# self.R_CamTag.append(Quaternion(cam_quat))
		# self.R_BaseGripper.append(Quaternion(grip_quat))
		# self.p_CamTag_Cam.append(np.array(cam_tra))
		# self.p_BaseGripper_Base.append(np.array(grip_tra))
		
		# changing reference frames to suit calibrateHandEye functionality in opencv
		self.R_TagCam.append(np.transpose(Quaternion(cam_quat).rotation_matrix))
		self.R_GripperBase.append(np.transpose(Quaternion(grip_quat).rotation_matrix))
		temp1 = np.matmul(-1*np.transpose(Quaternion(cam_quat).rotation_matrix), np.array(cam_tra))
		self.p_TagCam_Tag.append(temp1)
		temp2 = np.matmul(-1*np.transpose(Quaternion(grip_quat).rotation_matrix), np.array(grip_tra))
		self.p_GripperBase_Gripper.append(temp2)
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