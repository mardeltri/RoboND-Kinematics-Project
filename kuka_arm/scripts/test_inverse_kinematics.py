from mpmath import *
from sympy import *
import numpy as np

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
s = {alpha0:     0, a0:      0, d1:  0.75,
     alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
     alpha2:     0, a2:   1.25, d3:     0,
     alpha3: -pi/2, a3: -0.054, d4:  1.50,
     alpha4:  pi/2, a4:      0, d5:     0,
     alpha5: -pi/2, a5:      0, d6:     0,
     alpha6:     0, a6:      0, d7: 0.303, q7: 0}
#
# Define Modified DH Transformation matrix
#
#
# Create individual transformation matrices
#
T0_1 = T_matrix(q1,alpha0,a0,d1)
T0_1 = T0_1.subs(s)

T1_2 = T_matrix(q2,alpha1,a1,d2)
T1_2 = T1_2.subs(s)

T2_3 = T_matrix(q3,alpha2,a2,d3)
T2_3 = T2_3.subs(s)

T3_4 = T_matrix(q4,alpha3,a3,d4)
T3_4 = T3_4.subs(s)

T4_5 = T_matrix(q5,alpha4,a4,d5)
T4_5 = T4_5.subs(s)

T5_6 = T_matrix(q6,alpha5,a5,d6)
T5_6 = T5_6.subs(s)

T6_G = T_matrix(q7,alpha6,a6,d7)
T6_G = T6_G.subs(s)

T0_2 = simplify(T0_1 * T1_2)
T0_3 = simplify(T0_2 * T2_3)
T0_4 = simplify(T0_3 * T3_4)
T0_5 = simplify(T0_4 * T4_5)
T0_6 = simplify(T0_5 * T5_6)
T0_G = simplify(T0_6 * T6_G)

R_z = Matrix([[   cos(np.pi), -sin(np.pi),             0,   0],
      [   sin(np.pi),  cos(np.pi),             0,   0],
      [            0,           0,             1,   0],
      [            0,           0,             0,   1]])
R_y = Matrix([[ cos(np.pi/2),           0, sin(-np.pi/2),   0],
     [             0,           1,             0,   0],
     [-sin(-np.pi/2),           0, cos(-np.pi/2),   0],
     [             0,           0,             0,   1]])
R_corr = simplify(R_z*R_y)

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

R3_6 = R3_4*R4_5*R5_6
print(R3_6)
