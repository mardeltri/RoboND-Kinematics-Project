#! /usr/bin/env python

import numpy as np
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2
from sympy.matrices import Matrix

def T_matrix(theta_i, alpha_im1,a_im1, d_i):
    T_im1_i = Matrix([[                cos(theta_i),               -sin(theta_i),               0,               a_im1],
                      [ sin(theta_i)*cos(alpha_im1), cos(theta_i)*cos(alpha_im1), -sin(alpha_im1), -sin(alpha_im1)*d_i],
                      [ sin(theta_i)*sin(alpha_im1), cos(theta_i)*sin(alpha_im1),  cos(alpha_im1),  cos(alpha_im1)*d_i],
                      [                           0,                           0,               0,                   1]])
    return T_im1_i

### Create symbols for joint variabes
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta_i
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')


### Kuka KR210 ###
# DH Parameters
s = {alpha0:     0, a0:      0, d1:  0.75,
     alpha1: -pi/2., a1:   0.35, d2:     0, q2: q2-pi/2,
     alpha2:     0., a2:   1.25, d3:     0,
     alpha3: -pi/2., a3: -0.054, d4:   1.5,
     alpha4:  pi/2., a4:      0, d5:     0,
     alpha5: -pi/2., a5:      0, d6:     0,
     alpha6:     0, a6:      0, d7: 0.303, q7: 0}

#### Homogeneous Transforms
# base_link to link1
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

# Composition of Homogeneous Transforms
T0_2 = simplify(T0_1 * T1_2)
T0_3 = simplify(T0_2 * T2_3)
T0_4 = simplify(T0_3 * T3_4)
T0_5 = simplify(T0_4 * T4_5)
T0_6 = simplify(T0_5 * T5_6)
T0_G = simplify(T0_6 * T6_G)

# Correction needed to account of orientation difference between definition of
    # gripper_link in URDF versus DH convention
R_z = Matrix([[   cos(np.pi), -sin(np.pi),             0,   0],
	      [   sin(np.pi),  cos(np.pi),             0,   0],
              [            0,           0,             1,   0],
              [            0,           0,             0,   1]])
R_y = Matrix([[ cos(np.pi/2),           0, sin(-np.pi/2),   0],
             [             0,           1,             0,   0],
             [-sin(-np.pi/2),           0, cos(-np.pi/2),   0],
             [             0,           0,             0,   1]])
R_corr = simplify(R_z*R_y)

#### Numerically evaluate transforms (compare this with output of tf_echo!)
print("T0_1 = ", T0_1.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("T0_2 = ", T0_2.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("T0_3 = ", T0_3.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("T0_4 = ", T0_4.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("T0_5 = ", T0_5.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("T0_6 = ", T0_6.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("T0_G = ", T0_G.evalf(subs={q1: 0.21, q2: 0.2, q3: 0, q4: 0, q5: 0, q6: 0}))

# Total homogeneous transform between base_link and gripper_link with 
    # orientation correction applied 
T_total = simplify(T0_G * R_corr)
