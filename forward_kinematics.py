#!/usr/bin/env python3
#Daniel Perno RBE500 ROS Assignement 2 Forward Kinematics of a 3DOF Robot manipulator to calulcate position and orientation of end effector taking Joint 3 as the End Effector
import numpy as np	#Import numpy package
import rospy		#Import the Python Library for ROS

from math import pi, cos, sin, atan2, sqrt 				#Import the pi, cos, sin, atan2, sqrt messages from the math package

def forward_kinematics(joints):

	joint1 = joints[0]
	joint2 = joints[1]
	joint3 = joints[2]

	A = np.matrix([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
	thetas = np.array([joint1, joint2-1.249, joint3-1.249])
	dLengths = np.array([0.1039, 0,0])
	aLengths = np.array([0, 0.1581, 0.150])
	alphas = np.array([-pi/2, pi, pi/2])

	for i in range(len(joints)):
		
		tempA = np.matrix([[cos(thetas[i]), -sin(thetas[i])*cos(alphas[i]), sin(thetas[i])*sin(alphas[i]), aLengths[i]*cos(thetas[i])], [sin(thetas[i]), cos(thetas[i])*cos(alphas[i]), -cos(thetas[i])*sin(alphas[i]), aLengths[i]*sin(thetas[i])], [0, sin(alphas[i]), cos(alphas[i]), dLengths[i]], [0, 0, 0, 1]])
		A = np.matmul(A,tempA)
		print("step i ",i, "A: " , A)
		A=A
		
	print("A:", A)
	x=A[0,3]
	y=A[1,3]
	z=A[2,3]
	roll=atan2(A[2,1], A[2,2])
	pitch=atan2(-A[2,0], sqrt(np.square(A[2,1])+(np.square(A[2,2]))))
	yaw=atan2(A[1,0],A[0,0])
	print("X: " , x)
	print("Y: " , y)
	print("Z: " , z)
	print("Roll", roll)
	print("pitch", pitch)
	print("yaw", yaw)
	print("joints", thetas)
	print("Len joints", len(joints))
	return [x, y, z, roll, pitch, yaw]