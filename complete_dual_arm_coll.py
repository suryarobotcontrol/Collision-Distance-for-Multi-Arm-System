#! /usr/bin/env python3

import sys
import os
import math
import numpy as np
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Session_pb2, Base_pb2
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.Exceptions.KServerException import KServerException

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2
sys.path.append('/home/intelnuc/Desktop/demo_codes')

from fwd_kin_gen3lite import forward_kinematics
#from gpt2_trial_1 import detect_coll
from detect_line_collision import detect_coll
import matplotlib.pyplot as plt

TIMEOUT_DURATION = 20

c = 0

def check_for_end_or_abort(e):

    def check(notification, e=e):
        print("EVENT : " + Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()

    return check
def multiply_transform_matrices(matrix1, matrix2, matrix3):
    if matrix1.shape != (4, 4) or matrix2.shape != (4, 4):
        raise ValueError("Input matrices must be 4x4 transformation matrices.")
    T01 = np.matmul(matrix3, matrix1)
    T01 = np.linalg.inv(T01)
    result_matrix = np.matmul(T01, matrix2)
    print("result matrix1,", result_matrix )
    return result_matrix


def example_forward_kinematics(base, base2):
    global R01,R02,R03,R04,R05,R06,R0e,L01,L02,L03,L04,L05,L06,L0e,Tlr0,Tlr1,Tlr2,Tlr3,Tlr4,Tlr5,Tlr6,Tlre
    # Current arm's joint angles (in home position)
    try:
        input_joint_angles = base.GetMeasuredJointAngles()
    except KServerException as ex:
        print("Unable to get joint angles")
        print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
        print("Caught expected error: {}".format(ex))
        return None

    joint_angles_degrees = []
    for joint_angle in input_joint_angles.joint_angles:
        joint_angles_degrees.append(joint_angle.value)

    # print("Joint Angles in degrees", joint_angles_degrees)
    try:
        # print("Computing Forward Kinematics using joint angles...")
        pose = base.ComputeForwardKinematics(input_joint_angles)
    except KServerException as ex:
        print("Unable to compute forward kinematics")
        print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
        print("Caught expected error: {}".format(ex))
        return False

    joint_angles_radians = [(angle.value * math.pi) / 180 for angle in input_joint_angles.joint_angles]
    
    
    
    
    R01,R02,R03,R04,R05,R06,R0e = forward_kinematics(joint_angles_radians)
    # print("\nTotal Transformation Matrix for Right Arm:\n", R0e)
    # print("POSE is ", pose)
    try:
        input_joint_angles2 = base2.GetMeasuredJointAngles()
    except KServerException as ex:
        print("Unable to get joint angles")
        print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
        print("Caught expected error: {}".format(ex))
        return None

    joint_angles_degrees2 = []
    for joint_angle2 in input_joint_angles2.joint_angles:
        joint_angles_degrees2.append(joint_angle2.value)

    # print("Joint Angles in degrees for second Arm", joint_angles_degrees2)
    try:
        # print("Computing Forward Kinematics using joint angles...")
        pose2 = base2.ComputeForwardKinematics(input_joint_angles2)
    except KServerException as ex:
        print("Unable to compute forward kinematics")
        print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
        print("Caught expected error: {}".format(ex))
        return False

    joint_angles_radians2 = [(angle2.value * math.pi) / 180 for angle2 in input_joint_angles2.joint_angles]
    L01,L02,L03,L04,L05,L06,L0e = forward_kinematics(joint_angles_radians2)
    print()
    Tlr = np.array([[-1, 0, 0, 0.76], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    R00 = np.eye(4)
    Tlr0 = np.dot(Tlr,R00)
    Tlr1 = np.dot(Tlr,R01)
    Tlr2 = np.dot(Tlr,R02)
    Tlr3 = np.dot(Tlr,R03)
    Tlr4 = np.dot(Tlr,R04)
    Tlr5 = np.dot(Tlr,R05)
    Tlr6 = np.dot(Tlr,R06)
    Tlre = np.dot(Tlr,R0e)
    # print(f"Right Arm Joint1 with respect to Left Arm Base is {Tlr1}")
    # print(f"Right Arm Joint2 with respect to Left Arm Base is {Tlr2}")
    # print(f"Right Arm Joint3 with respect to Left Arm Base is {Tlr3}")
    # print(f"Right Arm Joint4 with respect to Left Arm Base is {Tlr4}")
    # print(f"Right Arm Joint5 with respect to Left Arm Base is {Tlr5}")
    # print(f"Right Arm Joint6 with respect to Left Arm Base is {Tlr6}")
    return True
def main():
    global R01,R02,R03,R04,R05,R06,R0e,L01,L02,L03,L04,L05,L06,L0e,Tlr0,Tlr1,Tlr2,Tlr3,Tlr4,Tlr5,Tlr6,Tlre,c
    R01=R02=R03=R04=R05=R06=R0e=L00=L01=L02=L03=L04=L05=L06=L0e=Tlr0=Tlr1=Tlr2=Tlr3=Tlr4=Tlr5=Tlr6=Tlre = np.eye(4)
    coll_matrix = np.zeros([4,4])
    
    
    sys.path.append('/home/intelnuc/Desktop/api_ws/src/api_pkg')
    from utilities import DeviceConnections

    connection = DeviceConnections()
    router, router2 = connection.routersTCP()

    # Create required services
    base = BaseClient(router)
    base2 = BaseClient(router2)
    base_cyclic = BaseCyclicClient(router)
    feedback = base_cyclic.RefreshFeedback()
    base_cyclic2 = BaseCyclicClient(router2)
    feedback2 = base_cyclic2.RefreshFeedback()


    # Example core
    success = True
    success &= example_forward_kinematics(base, base2)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    print(Tlr0[:3,3])
    L0_Ljoints = [L00[:3,3],L01[:3,3],L02[:3,3],L03[:3,3],L04[:3,3],L05[:3,3],L06[:3,3],L0e[:3,3]]
    L0_Rjoints = [Tlr0[:3,3],Tlr1[:3,3],Tlr2[:3,3],Tlr3[:3,3],Tlr4[:3,3],Tlr5[:3,3],Tlr6[:3,3],Tlre[:3,3]]
    for i in range(7):
        for j in range(7):
            if(i>2 and j>2):
                collision = detect_coll(L0_Ljoints[i],L0_Ljoints[i+1],L0_Rjoints[j],L0_Rjoints[j+1])
                coll_matrix [i-3,j-3] = collision
        ax.plot([L0_Rjoints[i][0],L0_Rjoints[i+1][0]], [L0_Rjoints[i][1],L0_Rjoints[i+1][1]], [L0_Rjoints[i][2],L0_Rjoints[i+1][2]], label=f'right_joint{i + 1}', color=(i/7,1-i/7,1-i/7), linewidth=4.0)
        ax.plot([L0_Ljoints[i][0],L0_Ljoints[i+1][0]], [L0_Ljoints[i][1],L0_Ljoints[i+1][1]], [L0_Ljoints[i][2],L0_Ljoints[i+1][2]], label=f'left_joint{i + 1}', color=(1-i/7,1-i/7,i/7), linewidth = 4.0)
    

    ax.scatter(L0_Rjoints[7][0],L0_Rjoints[7][1],L0_Rjoints[7][2], color='g', marker='o', s=100, label='right_end_effector')
    ax.scatter(L0_Ljoints[7][0], L0_Ljoints[7][1], L0_Ljoints[7][2], color='r', marker='o', s=100, label='left_end_effector')

    print( "J3,    J4,    J5,    J6")
    print(coll_matrix)

    all_joints = np.vstack((L0_Ljoints, L0_Rjoints))
    x_min, x_max = all_joints[:,0].min(), all_joints[:,0].max()
    y_min, y_max = all_joints[:,1].min(), all_joints[:,1].max()
    z_min, z_max = all_joints[:,2].min(), all_joints[:,2].max()

    # Find the maximum range
    max_range = max(x_max - x_min, y_max - y_min, z_max - z_min)
    mid_x = (x_max + x_min) / 2
    mid_y = (y_max + y_min) / 2
    mid_z = (z_max + z_min) / 2

    # Set limits with equal scaling
    ax.set_xlim(mid_x - max_range / 2, mid_x + max_range / 2)
    ax.set_ylim(mid_y - max_range / 2, mid_y + max_range / 2)
    ax.set_zlim(mid_z - max_range / 2, mid_z + max_range / 2)
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.legend()
    plt.show()

    connection.close()

    return 0 if success else 1

if __name__ == "__main__":
    exit(main())
