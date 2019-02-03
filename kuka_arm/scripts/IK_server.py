#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
import numpy as np
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

### Define functions for Rotation Matrices about x, y, and z given specific angle.

def rot_x(q):
    R_x = Matrix([[ 1,              0,        0],
              [ 0,        cos(q), -sin(q)],
              [ 0,        sin(q),  cos(q)]])
    
    return R_x
    
def rot_y(q):              
    R_y = Matrix([[ cos(q),        0,  sin(q)],
              [       0,        1,        0],
              [-sin(q),        0,  cos(q)]])
    
    return R_y

def rot_z(q):    
    R_z = Matrix([[ cos(q), -sin(q),        0],
              [ sin(q),  cos(q),        0],
              [ 0,              0,        1]])
    
    return R_z

def T_matrix(theta_i, alpha_im1,a_im1, d_i):
    T_im1_i = Matrix([[                cos(theta_i),               -sin(theta_i),               0,               a_im1],
                      [ sin(theta_i)*cos(alpha_im1), cos(theta_i)*cos(alpha_im1), -sin(alpha_im1), -sin(alpha_im1)*d_i],
                      [ sin(theta_i)*sin(alpha_im1), cos(theta_i)*sin(alpha_im1),  cos(alpha_im1),  cos(alpha_im1)*d_i],
                      [                           0,                           0,               0,                   1]])
    return T_im1_i

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
	#
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta_i
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
	#
	# Create Modified DH parameters
	#
        DH_param = {alpha0:     0, a0:      0, d1:  0.75, q1: q1,
                    alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
                    alpha2:     0, a2:   1.25, d3:     0, q3: q3,
                    alpha3: -pi/2, a3: -0.054, d4:  1.50, q4: q4,
                    alpha4:  pi/2, a4:      0, d5:     0, q5: q5,
                    alpha5: -pi/2, a5:      0, d6:     0, q6: q6,
                    alpha6:     0, a6:      0, d7: 0.303, q7: 0}
	#
	# Define Modified DH Transformation matrix
	#
	#
	# Create individual transformation matrices
	#
        T0_1 = T_matrix(q1,alpha0,a0,d1)
        T0_1 = T0_1.subs(DH_param)

        T1_2 = T_matrix(q2,alpha1,a1,d2)
        T1_2 = T1_2.subs(DH_param)

        T2_3 = T_matrix(q3,alpha2,a2,d3)
        T2_3 = T2_3.subs(DH_param)

        T3_4 = T_matrix(q4,alpha3,a3,d4)
        T3_4 = T3_4.subs(DH_param)
        
        T4_5 = T_matrix(q5,alpha4,a4,d5)
        T4_5 = T4_5.subs(DH_param)

        T5_6 = T_matrix(q6,alpha5,a5,d6)
        T5_6 = T5_6.subs(DH_param)

        T6_G = T_matrix(q7,alpha6,a6,d7)
        T6_G = T6_G.subs(DH_param)
    
        T0_G = T0_1* T1_2* T2_3*T3_4*T4_5*T5_6*T6_G

      	R_z = Matrix([[-1,  0,  0,   0],
                      [ 0, -1,  0,   0],
                      [ 0,  0,  1,   0],
                      [ 0,  0,  0,   1]])
        R_y = Matrix([[ 0,  0, -1,   0],
                      [ 0,  1,  0,   0],
                      [ 1,  0,  0,   0],
                      [ 0,  0,  0,   1]])
      	R_corr = R_z*R_y

        T_total = simplify(T0_G * R_corr)	
	#
	# Extract rotation matrices from the transformation matrices
	#
	#
        R0_1 = T0_1[0:3,0:3]
        R1_2 = T1_2[0:3,0:3]
        R2_3 = T2_3[0:3,0:3]
        R3_4 = T3_4[0:3,0:3]
        R4_5 = T4_5[0:3,0:3]
        R5_6 = T5_6[0:3,0:3]

        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
            R_rpy = rot_z(yaw)*rot_y(pitch)*rot_x(roll)*R_corr[0:3,0:3]
	    #
	    # Calculate joint angles using Geometric IK method
	    #
            # Calculate wrist position
            nx = R_rpy[0,2]
            ny = R_rpy[1,2]
	        nz = R_rpy[2,2]
            #
            wx = px - (0.303)*nx
            wy = py - (0.303)*ny
            wz = pz - (0.303)*nz
            ###
            theta1 = atan2(wy,wx)
            
            # Calculate theta 2 and theta 3
            # Geometric parameters
            A = 1.501
            B = sqrt(pow((sqrt(wx*wx+wy*wy)-0.35),2) + pow((wz-0.75),2))
            C = 1.25

            a = acos((B**2+C**2-A**2)/(2*B*C))
            b = acos((A**2+C**2-B**2)/(2*A*C))
            
            theta2 = pi/2 - a - atan2(wz-0.75,sqrt(wx*wx+wy*wy)-0.35)
            theta3 = pi/2 - b - 0.036
			
            # Calculate theta 4,5 and 6
            R0_3 = T0_1[0:3,0:3]*T1_2[0:3,0:3]*T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs = {q1: theta1, q2: theta2, q3: theta3})
            R3_6 = R0_3.inv("LU")*R_rpy
            
			theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
            
			# Choose the  correct solution for theta 4 and theta 6
			if (theta5 > pi) :
                theta4 = atan2(-R3_6[2,2], R3_6[0,2])
                theta6 = atan2(R3_6[1,1],-R3_6[1,0])
            else:
                theta4 = atan2(R3_6[2,2], -R3_6[0,2])
                theta6 = atan2(-R3_6[1,1],R3_6[1,0])
            
			# Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
