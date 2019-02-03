## Project: Kinematics Pick & Place

[//]: # (Image References)

[ForwardKinematcs]: ./misc_images/ForwardKinematcs.png
[URDF]: ./misc_images/URDF.png
[DHRF]: ./misc_images/DHRF.png
[TransformRotations]: ./misc_images/TransformRotations.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

In this section how to obtain the DH parameters table will be addressed. These parameters will help us to describe the transformation
matrices to relate the end effector position and the joint angles.

First, in order to understand how the Kuka KR210 can move the forward kinematics demo can be launched.

![alt text][ForwardKinematcs]

Moving the sliders in joint_state_publisher it can be understood the robot workspace as well as the joint limits.

Before obtaining the DH parameters we need to identify the joint positions with respect to each other, thus defining
the axes of each joint. This geometrical data can be obtained from the kr210.urdf.xacro file.

For example, from this line we know that joint 1 is 0.33 m above base link and at the same x and y position.

```
<joint name="joint_1" type="revolute">
    <origin xyz="0 0 0.33" rpy="0 0 0"/>
```
![alt text][URDF]

Next, following the steps explained in Lesson 14, we can assign reference frames to our manipulator's links.
![alt text][DHRF]

Taking into account the geometrical data obtained from the kr210.urdf.xacro file, the reference frames defined 
above and the definition of the following variables, it is not difficult to derived the DH parameters.
* alpha(i-1) (twist angle) = angle between Z(i-1) and Z(i) measured about X(i-1) in a right-hand sense.
* a(i-1) (link legth) = distance from  Z(i-1) and Z(i) measured along X(i-1) where X(i-1) is
perpendicular to both Z(i-1) and Z(i)
* d(i) (link offset) = signed distance from X(i-1) to X(i) measured along Z(i). 
* theta(i) (joint angle) = angle between X(i-1) to X(i) measured about Z(i) in right-hand sense.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | q2 - pi/2 
2->3 | 0 | 1.25 | 0 | q3
3->4 |  - pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.
Considering the previous definitions, the homogeneous transform from frame i-1 to frame i is cconstractud as a sequence of four basic transformations, two rotations and two transalations, thus:
![alt text][DHRF]




#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


