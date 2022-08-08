# -*- coding: utf-8 -*-
"""
Created on Tue Jul 12 15:45:07 2022

@author: Longfei
"""
import rosbag
import bagpy
from bagpy import bagreader
import pandas as pd
import cv2
import numpy as np
from math import *
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import glob


#  tag0 to camera
  # translation: 
  #   x: -0.15724111518307224
  #   y: 0.15807698231204478
  #   z: 1.0343631981241868
  # rotation: 
  #   x: 0.7965683915642364
  #   y: -0.07828504847614697
  #   z: 0.030642908091957487
  #   w: 0.598674586841233]
  
#   frame_id: "blue_base"
# child_frame_id: "blue_tool0_controller"
# transform: 
#   translation: 
#     x: -0.40735711824383947
#     y: -0.1648734851802117
#     z: 1.0899091439204274
#   rotation: 
#     x: -0.44530293818298594
#     y: -0.40381132216350485
#     z: 0.5536076234276421
#     w: 0.5763335046836681]



# def XYZW_to_R(XYZW):
#     x = XYZW[0]; y = XYZW[1]; z = XYZW[2]; w = XYZW[3]; 
#     Rx = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
#     Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
#     Rz = np.array([[cos(z), -sin(z), 0], [sin(z), cos(z), 0], [0, 0, 1]])
#     R = Rz@Ry@Rx
#     return R

def quaternion_rotation_matrix(Q):
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

def inverse_RT(R, T):
    RT = np.zeros((4,4))
    RT[0:3, 0:3] = R
    RT[3,3] = 1
    RT[0:3, 3] = T.transpose()
    RT_inv = np.linalg.inv(RT)
    R_inv = RT_inv[0:3, 0:3]
    T_inv = RT_inv[0:3, 3].transpose()
    return R_inv, T_inv
   
def data_to_RT_matrix(line_now):
    T_now = np.array( line_now[4:7] )
    R_now_quaternions = np.array( line_now[7:11] )
    R_now = R.from_quat(R_now_quaternions).as_matrix()
    RT = np.zeros((4,4))
    RT[0:3, 0:3] = R_now
    RT[0:3,3] = T_now
    RT[3,3] = 1
    return RT

    
def extract_data(aaa_all):
    tag_name = 'tag_0'
    end_name = 'panda_link7'
    
    R_all_end_to_base = []
    T_all_end_to_base = []
    time_all_end_to_base = []
    indx_all_end_to_base = []
    
    R_all_tag_to_cam = []
    T_all_tag_to_cam = []
    time_all_tag_to_cam = []
    indx_all_tag_to_cam = []
    
    for i in range(len(aaa_all)):
        
        if aaa_all[i][0][2] == tag_name:
            line_now = aaa_all[i][0]
            T_now = np.array( line_now[4:7] )
            R_now_quaternions = np.array( line_now[7:11] )
            # R_now = quaternion_rotation_matrix(R_now_quaternions)
            R_now = R.from_quat(R_now_quaternions).as_matrix()
            # R_now, T_now = inverse_RT(R_now, T_now)
            time_now = np.array( line_now[0:2] )
            name_now = line_now[2]
            R_all_tag_to_cam.append(R_now)
            T_all_tag_to_cam.append(T_now)
            time_all_tag_to_cam.append(time_now)
            indx_all_tag_to_cam.append(i)
        
        
            for j in range(i,i+4):
                if len(aaa_all[j])>5:
                    nearby_rows = aaa_all[j]
                    break
            
            for k in range(len(nearby_rows)):
                current_row = nearby_rows[k]
                if current_row[2] == 'panda_link7':
                    lin7to6 = current_row
                if current_row[2] == 'panda_link6':
                    lin6to5 = current_row      
                if current_row[2] == 'panda_link5':
                    lin5to4 = current_row  
                if current_row[2] == 'panda_link4':
                    lin4to3 = current_row  
                if current_row[2] == 'panda_link3':
                    lin3to2 = current_row  
                if current_row[2] == 'panda_link2':
                    lin2to1 = current_row  
                if current_row[2] == 'panda_link1':
                    lin1to0 = current_row  

            lin7to6_RT = data_to_RT_matrix(lin7to6)
            lin6to5_RT = data_to_RT_matrix(lin6to5)
            lin5to4_RT = data_to_RT_matrix(lin5to4)
            lin4to3_RT = data_to_RT_matrix(lin4to3)
            lin3to2_RT = data_to_RT_matrix(lin3to2)
            lin2to1_RT = data_to_RT_matrix(lin2to1)
            lin1to0_RT = data_to_RT_matrix(lin1to0)
            RT_7to0 = lin1to0_RT.dot(lin2to1_RT.dot(lin3to2_RT.dot(lin4to3_RT.dot(lin5to4_RT.dot(lin6to5_RT.dot(lin7to6_RT))))))
            # print(RT_7to0)
            R_now1 = RT_7to0[0:3, 0:3]
            T_now1 =  RT_7to0[0:3, 3]
            time_now2 = np.array( lin1to0[0:2] )
            j0 = i + 4
            
            R_all_end_to_base.append(R_now1)
            T_all_end_to_base.append(T_now1)
            time_all_end_to_base.append(time_now2)
            indx_all_end_to_base.append(j0)




                    
            # # find nearby end rows
            # seg = 13
            # # nearby_rows = aaa_all[   np.max((0, i-seg)): np.min((i+seg, len(aaa_all)))]
            # nearby_rows = aaa_all[i: i+13]
            # # min_time = 1000000
            # # for j in range(0, len(nearby_rows)):
            # #     line_now1 = nearby_rows[j]
            # #     if len(line_now1)<10:
            # #         continue
            # #     time_now1 = np.array( line_now1[0:2] )
            # #     name_now1 = line_now1[2]
            # #     time_diff = abs((time_now1[0] - time_now[0]) * 1000 + (time_now1[1] - time_now[1])/100000)   # ms
            # #     if name_now1==end_name and time_diff<min_time:
            # #         min_time = time_diff 
            # #         T_now1 = np.array( line_now1[4:7] )
            # #         R_now1_quaternions1 = np.array( line_now1[7:11] )
            # #         R_now1 = R.from_quat(R_now1_quaternions1).as_matrix()
            # #         time_now2 = time_now1
            # #         j0 = j
            # #         # R_now1, T_now1 = inverse_RT(R_now1, T_now1)
            # print(nearby_rows)
            
            # lin7to6 = nearby_rows[10]
            # lin6to5 = nearby_rows[9]       
            # lin5to4 = nearby_rows[8] 
            # lin4to3 = nearby_rows[7] 
            # lin3to2 = nearby_rows[6] 
            # lin2to1 = nearby_rows[5] 
            # lin1to0 = nearby_rows[4] 

                    
    return R_all_end_to_base, T_all_end_to_base, np.array(time_all_end_to_base), np.array(indx_all_end_to_base),  R_all_tag_to_cam, T_all_tag_to_cam,  np.array(time_all_tag_to_cam),  np.array(indx_all_tag_to_cam)


# def read_bag_static():
#     b = bagreader('calibrationPandaArmStops.bag')
#     # csvfiles = []
#     # for t in b.topics:
#     #     data = b.message_by_topic(t)
#     #     csvfiles.append(data)
#     # list_id = ['blue_tool0_controller', 'tag_0', 'tag_1']
#     aaa_all = []
#     bag = rosbag.Bag('calibrationPandaArmStops.bag')
#     info = bag.get_type_and_topic_info()
#     print(info)
#     for topic, msg, t in bag.read_messages(topics=['/tf_static']):
#         # print(msg)
#         aaa = []
#         for j in range(len(msg.transforms)):
#             # if msg.transforms[j].child_frame_id in list_id:
#             msg_now = msg.transforms[j];
#             aaa.append(msg_now.header.stamp.secs)
#             aaa.append(msg_now.header.stamp.nsecs)
#             aaa.append(msg_now.child_frame_id)
#             aaa.append(msg_now.header.frame_id)
#             aaa.append(msg_now.transform.translation.x)
#             aaa.append(msg_now.transform.translation.y)
#             aaa.append(msg_now.transform.translation.z)
#             aaa.append(msg_now.transform.rotation.x)
#             aaa.append(msg_now.transform.rotation.y)
#             aaa.append(msg_now.transform.rotation.z)
#             aaa.append(msg_now.transform.rotation.w)
#             # sdir='d://'
#             # geo.to_csv(sdir+"georesult1.csv")
#         # print(aaa)
#         aaa_all.append(aaa)
#         # geo=pd.read_csv(b.message_by_topic("/tf"))
#         # sdir='d://'
#         # geo.to_csv(sdir+"georesult1.csv")
#     bag.close()
#     return aaa_all


def read_bag():
    
    bag_fd = './calibrationPandaArmStops/'
    bag_files = []
    for filename in glob.glob(bag_fd + '*.bag'): 
        bag_files.append(filename) 
    
    
    aaa_all = []
    for bb in range(len(bag_files)):
        # b = bagreader(bag_files[bb])
        # csvfiles = []
        # for t in b.topics:
        #     data = b.message_by_topic(t)
        #     csvfiles.append(data)
        # list_id = ['panda_EE', 'tag_0', 'tag_1']

        bag = rosbag.Bag(bag_files[bb])
        info = bag.get_type_and_topic_info()
        print(info)
        for topic, msg, t in bag.read_messages(topics=['/tf']):
            # print(msg)
            aaa = []
            if len(msg.transforms)==2:  # remove EE and K useless ones
                continue
            for j in range(len(msg.transforms)):
                # if msg.transforms[j].child_frame_id in list_id:
                    # try:
                msg_now = msg.transforms[j];
                one_msg = []
                # print(msg_now)
                one_msg.append(msg_now.header.stamp.secs)
                one_msg.append(msg_now.header.stamp.nsecs)
                one_msg.append(msg_now.child_frame_id)
                one_msg.append(msg_now.header.frame_id)
                one_msg.append(msg_now.transform.translation.x)
                one_msg.append(msg_now.transform.translation.y)
                one_msg.append(msg_now.transform.translation.z)
                one_msg.append(msg_now.transform.rotation.x)
                one_msg.append(msg_now.transform.rotation.y)
                one_msg.append(msg_now.transform.rotation.z)
                one_msg.append(msg_now.transform.rotation.w)
                aaa.append(one_msg)
                    # except:
                    #     print('read msg error: ')
                    #     print(msg)
                # sdir='d://'
                # geo.to_csv(sdir+"georesult1.csv")
            # print(aaa)
            aaa_all.append(aaa)
            # geo=pd.read_csv(b.message_by_topic("/tf"))
            # sdir='d://'
            # geo.to_csv(sdir+"georesult1.csv")
        bag.close()
    return aaa_all



aaa_all = read_bag()

# # # bbb_all = read_bag_static()
R_all_end_to_base, T_all_end_to_base, time_all_end_to_base, indx_all_end_to_base,  R_all_tag_to_cam, T_all_tag_to_cam, \
    time_all_tag_to_cam,  indx_all_tag_to_cam = extract_data(aaa_all)



    
# # temp1 = np.array(T_all_end_to_base)
# # # Creating plot
# # fig = plt.figure()
# # ax = plt.axes(projection ="3d")
# # # Creating plot
# # ax.scatter3D(temp[:,0], temp[:,1], temp[:,2])
# # # ax.title("3D scatter plot end to base")
# # ax.set_xlabel('X Label')
# # ax.set_ylabel('Y Label')
# # ax.set_zlabel('Z Label')
# # ax.set_title('3d model')
# # ax.set_xlim((-1, 1)); ax.set_ylim((-1, 1)); ax.set_zlim((0, 2)); 
# # # show plot
# # plt.show()

data_len = (np.linspace(20, len(T_all_end_to_base)-20, 20)).astype(int)
R_end_to_base = [R_all_end_to_base[i] for i in data_len]
T_end_to_base = [T_all_end_to_base[i] for i in data_len]
R_tag_to_cam = [R_all_tag_to_cam[i] for i in data_len]
T_tag_to_cam = [T_all_tag_to_cam[i] for i in data_len]


# temp = np.array(T_all_tag_to_cam)
# temp1 = np.array(T_all_end_to_base)
temp2 = np.array(T_tag_to_cam)
temp3 = np.array(T_end_to_base)
# Creating plot
fig = plt.figure()
ax = plt.axes(projection ="3d")
# Creating plot
# ax.scatter3D(temp[:,0], temp[:,1], temp[:,2], label="tag2cam")
# ax.scatter3D(temp1[:,0], temp1[:,1], temp1[:,2], label="end2base")
ax.scatter3D(temp2[:,0], temp2[:,1], temp2[:,2],  s=50, label="sap tag2cam")
ax.scatter3D(temp3[:,0], temp3[:,1], temp3[:,2],  s=50, label="sap end2base")
for k in range(len(temp2)):
    ax.plot([temp2[k,0], temp3[k,0]], [temp2[k,1], temp3[k,1]], [temp2[k,2], temp3[k,2]], color='black')
# ax.plot(  [[temp2[0,0], temp3[0,0]] , [temp2[0,1],[temp3[0,1]] , [ temp2[0,2], temp3[0,2] ]]   )
# ax.title("3D scatter plot tag to cam")
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
ax.set_title('3d model')
ax.set_xlim((-1, 0)); ax.set_ylim((-0.5, 0.5)); ax.set_zlim((0, 1.5)); 
plt.legend(loc="upper right")
# show plot
plt.show()


R,T = cv2.calibrateHandEye(R_end_to_base,T_end_to_base, R_tag_to_cam, T_tag_to_cam)
# 
print(R)
print(T)




RT = np.zeros((4,4))
RT[0:3, 0:3] = R
RT[0:3,3] = list(T)
RT[3,3] = 1
RT_inv = np.linalg.inv(RT)

















































# # georesout=b.message_by_topic("/tf_static")
# # geo=pd.read_csv(georesout)
# # sdir='d://'
# # geo.to_csv(sdir+"georesult1.csv")
# # geo


# # # replace the topic name as per your need
# # LASER_MSG = b.message_by_topic('/tf_static')
# # LASER_MSG
# # df_laser = pd.read_csv(LASER_MSG)
# # df_laser # prints laser data in the form of pandas dataframe

# # times = df_laser.Time
# # transforms = df_laser.transforms


# # R_all_end_to_base_1 = [   np.array([  [1, 0, 0], [0, 1, 0], [0, 0, 1]  ]), np.array([ [1, 0, 0], [0, 1, 0], [0, 0, 1]  ]) , np.array([ [1, 0, 0], [0, 1, 0], [0, 0, 1]  ])   ]
# # T_all_end_to_base_1 = [   np.array([0.7, 1.2, 2]), np.array([0.9, 1.4, 2.2]), np.array([1.1, 1.6, 2.4])  ]

# # R_all_chess_to_cam_1 = [  np.array( [  [1, 0, 0], [0, 1, 0], [0, 0, 1]  ]), np.array([ [1, 0, 0], [0, 1, 0], [0, 0, 1]  ]), np.array([ [1, 0, 0], [0, 1, 0], [0, 0, 1]  ])     ]
# # T_all_chess_to_cam_1 = [ np.array([-0.9, 1.4, 2.2]), np.array([-1.1, 1.6, 2.4]) , np.array([-1.3, 1.8, 2.6]) ]


# # R,T = cv2.calibrateHandEye(R_all_end_to_base,T_all_end_to_base,R_all_tag_to_cam,T_all_tag_to_cam)





