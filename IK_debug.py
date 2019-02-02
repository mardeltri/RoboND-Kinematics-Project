from sympy import *
from time import time
from mpmath import radians
import tf
import numpy as np
'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[[[-0.078724,0.182528,3.09863],
                  [-0.560845,-0.201777,0.0925041,0.797611]],
                  [-0.35187,0.069237,3.0325],
                  [2.95,-0.65,-0.42,3.73,1.77,-0.99]],
              5:[]}

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

def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 
	
    ## Insert IK code here!
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
    print("Created individual transformation matrices")

    T0_G = T0_1* T1_2* T2_3*T3_4*T4_5*T5_6*T6_G
    print("Simplified transformation matrices")

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
    print("Simplified T_total")
    
    #
    # Extract rotation matrices from the transformation matrices
    #
    #
    R0_1 = T0_1[0:3,0:3]
    print("R0_1")
    print(R0_1)
    R1_2 = T1_2[0:3,0:3]
    R2_3 = T2_3[0:3,0:3]
    R3_4 = T3_4[0:3,0:3]
    R4_5 = T4_5[0:3,0:3]
    R5_6 = T5_6[0:3,0:3]
    
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
#    print(wx,wy)
#    theta1 = atan2(wy,wx).evalf()
#    print("Theta1:",theta1)
#    # Calculate theta 2 and theta 3
#    # Geometric parameters
#    A = 1.501
#    # Compute B
#    r01_0 = Matrix([ 0, 0, 0.75])
#    r02_0 = Matrix([ 0.35*cos(theta1), 0.35*sin(theta1), 0])
#    rwc_0 = Matrix([ wx, wy, wz])
#    Bv = rwc_0 - r01_0 - r02_0
#    B = sqrt(Bv[0,0]**2+Bv[1,0]**2+Bv[2,0]**2)
#    C = 1.25
#    print("Computed A,B,C")
#    a = acos((B**2+C**2-A**2)/(2*B*C))
#    b = acos((A**2+C**2-B**2)/(2*A*C))
#    d = atan2(Bv[2,0],(sqrt(wx**2+wy**2)-0.35))

#    theta2 = pi/2-a-d
#    print("Theta2:",theta2.evalf())
#    print("Computed theta2")
#    #psi = atan2(0.054,1.5)
#    psi = 0.036
#    print(psi)
#    theta3 = (pi/2-b-psi)
#    print("Theta3:",theta3.evalf())
#    print("Computed theta3",theta3.evalf())
    
    theta1 = atan2(wy,wx)
            
    # Calculate theta 2 and theta 3
    # Geometric parameters
    #A = 1.501
    # Compute B
    r01_0 = Matrix([ 0, 0, 0.75])
    r02_0 = Matrix([ 0.35*cos(theta1), 0.35*sin(theta1), 0])
    rwc_0 = Matrix([ wx, wy, wz])
    Bv = rwc_0 - r01_0 - r02_0
    #B = sqrt(Bv[0,0]**2+Bv[1,0]**2+Bv[2,0]**2)
    #C = 1.25
    A = 1.501
    B = sqrt(pow((sqrt(wx*wx+wy*wy)-0.35),2) + pow((wz-0.75),2))
    C = 1.25

    a = acos((B**2+C**2-A**2)/(2*B*C))
    b = acos((A**2+C**2-B**2)/(2*A*C))
    d = atan2(Bv[2,0],(sqrt(wx**2+wy**2)-0.35))
    
    theta2 = pi/2 - a - atan2(wz-0.75,sqrt(wx*wx+wy*wy)-0.35)
    theta3 = pi/2 - b - 0.036

    #theta2 = pi/2-a-d

    #psi = atan2(0.054,1.5)
    #psi = 0.036
    
    #theta3 = simplify(pi/2-b-psi)
    print("Theta1: %04.8f" % theta1.evalf())
    print("Theta2: %04.8f" % theta2.evalf())
    print("Theta3: %04.8f" % theta3.evalf())
    # Calculate theta 4,5 and 6
    R0_3 = T0_1[0:3,0:3]*T1_2[0:3,0:3]*T2_3[0:3,0:3]
    R0_3 = R0_3.evalf(subs = {q1: theta1, q2: theta2, q3: theta3})
    R3_6 = R0_3.inv("LU")*R_rpy
    theta4 = atan2(R3_6[2,2],-R3_6[0,2])
    print("Theta4: %04.8f" % theta4.evalf())
    theta5 = atan2(sqrt(R3_6[1,0]*R3_6[1,0] + R3_6[1,1]*R3_6[1,1]),R3_6[1,2])
    print("Theta5: %04.8f" % theta5.evalf())
    theta6 = atan2(-R3_6[1,1],R3_6[1,0])
    print("Theta6: %04.8f" % theta6.evalf())
    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    T_total = T_total.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
     
    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [wx,wy,wz] # <--- Load your calculated WC values in this array
    your_ee = [T_total[0,3],T_total[1,3],T_total[2,3]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 2

    test_code(test_cases[test_case_number])
