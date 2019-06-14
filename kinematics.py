#!/usr/bin/env python

import numpy as np 
import math

zero_col = np.zeros((3,1))
print(zero_col)
class Kinematics:
	def __init__(self):
		self.x=0.35
		self.y=0.35
		self.z=0.5
		self.d = 0.7
		self.w = 0.7
		self.h = 0.7

		self.x_dot =  0.1
		self.y_dot =  0.1
		self.z_dot =  0

	def forward_kinematics(self):

		# self.theta_0 = np.arctan2(self.y,self.x)
		# self.phi_0 = np.arctan2(self.z,self.x)
		# self.l_0 = (self.x)/(np.cos(self.theta_0)*np.cos(self.phi_0))

		# self.theta_1 = np.arctan2((self.y-self.d),self.x)
		# self.phi_1 = np.arctan2(self.z,self.x)
		# self.l_1 = (self.x)/(np.cos(self.theta_1)*np.cos(self.phi_1))

		# self.theta_2 = np.arctan2((self.y-self.d),(self.x-self.w))
		# self.phi_2 = np.arctan2(self.z,(self.x-self.w))
		# self.l_2 = (self.x-self.w)/(np.cos(self.theta_2)*np.cos(self.phi_2))

		# self.theta_3 = np.arctan2((self.y),(self.x-self.w))
		# self.phi_3 = np.arctan2(self.z,(self.x-self.w))
		# self.l_3 = (self.x-self.w)/(np.cos(self.theta_3)*np.cos(self.phi_3))

		self.l0 = math.sqrt((self.x)**2+(self.y)**2+(self.h-self.z)**2)
		self.l1 = math.sqrt((self.x-self.d)**2+(self.y)**2+(self.h-self.z)**2)
		self.l2 = math.sqrt((self.x-self.d)**2+(self.y-self.w)**2+(self.h-self.z)**2)
		self.l3 = math.sqrt((self.x)**2+(self.y-self.w)**2+(self.h-self.z)**2)

		self.phi0 = np.arcsin((self.h-self.z)/self.l0)
		self.phi1 = np.arcsin((self.h-self.z)/self.l1)
		self.phi2 = np.arcsin((self.h-self.z)/self.l2)
		self.phi3 = np.arcsin((self.h-self.z)/self.l3)

		self.theta0 = np.arccos(self.x/(self.l0*np.cos(np.arcsin((self.h-self.z)/self.l0))))
		self.theta1 = np.arccos((self.x-self.d)/(self.l1*np.cos(np.arcsin((self.h-self.z)/self.l1))))
		self.theta2 = np.arccos((self.x-self.d)/(self.l2*np.cos(np.arcsin((self.h-self.z)/self.l2))))+np.pi/2
		self.theta3 = np.arccos(self.x/(self.l3*np.cos(np.arcsin((self.h-self.z)/self.l3))))+np.pi/2+np.pi


		print("thetas")
		print(self.theta0)
		print(self.theta1)
		print(self.theta2)
		print(self.theta3)
		print("phis")
		print(self.phi0)
		print(self.phi1)
		print(self.phi2)
		print(self.phi3)
		print("lengths")
		print(self.l0)
		print(self.l1)
		print(self.l2)
		print(self.l3)



	def get_Jacobians(self):
		# self.J0=np.array([[  np.cos(self.phi0)*np.sin(self.theta0),  self.l0*np.cos(self.phi0)*np.cos(self.theta0), -self.l0*np.sin(self.phi0)*np.sin(self.theta0)],
		# 				  [  np.cos(self.phi0)*np.cos(self.theta0), -self.l0*np.cos(self.phi0)*np.sin(self.theta0), -self.l0*np.sin(self.phi0)*np.cos(self.theta0)],
		# 				  [  np.sin(self.phi0)                    ,  0                                            ,  self.l0*np.cos(self.phi0)                    ]])
		# self.J1=np.array([[ -np.cos(self.phi1)*np.sin(self.theta1), -self.l1*np.cos(self.phi1)*np.cos(self.theta1),  self.l1*np.sin(self.phi1)*np.sin(self.theta1)],
		# 				  [  np.cos(self.phi1)*np.cos(self.theta1), -self.l1*np.cos(self.phi1)*np.sin(self.theta1), -self.l1*np.sin(self.phi1)*np.cos(self.theta1)], 
		# 				  [  np.sin(self.phi1)                    ,  0                                            ,  self.l1*np.cos(self.phi1)                    ]])
		# self.J2=np.array([[ -np.cos(self.phi2)*np.sin(self.theta2), -self.l2*np.cos(self.phi2)*np.cos(self.theta2),  self.l2*np.sin(self.phi2)*np.sin(self.theta2)], 
		# 				  [ -np.cos(self.phi2)*np.cos(self.theta2),  self.l2*np.cos(self.phi2)*np.sin(self.theta2),  self.l2*np.sin(self.phi2)*np.cos(self.theta2)], 
		# 				  [  np.sin(self.phi2)                    ,  0                                            ,  self.l2*np.cos(self.phi2)                    ]])
		# self.J3=np.array([[  np.cos(self.phi3)*np.sin(self.theta3),  self.l3*np.cos(self.phi3)*np.cos(self.theta3), -self.l3*np.sin(self.phi3)*np.sin(self.theta3)], 
		# 				  [ -np.cos(self.phi3)*np.cos(self.theta3),  self.l3*np.cos(self.phi3)*np.sin(self.theta3),  self.l3*np.sin(self.phi3)*np.cos(self.theta3)], 
		# 				  [  np.sin(self.phi3)                    ,  0                                            ,  self.l3*np.cos(self.phi3)                    ]]) 
		# ##best so far
		# self.J0=np.array([[  np.cos(self.phi0)*np.cos(self.theta0), -self.l0*np.cos(self.phi0)*np.sin(self.theta0), -self.l0*np.sin(self.phi0)*np.cos(self.theta0)],
		# 				  [  np.cos(self.phi0)*np.sin(self.theta0),  self.l0*np.cos(self.phi0)*np.cos(self.theta0), -self.l0*np.sin(self.phi0)*np.sin(self.theta0)],
		# 				  [  np.sin(self.phi0)*np.cos(self.theta0), -self.l0*np.sin(self.phi0)*np.sin(self.theta0),  self.l0*np.cos(self.phi0)*np.cos(self.theta0)]])
		# self.J1=np.array([[  np.cos(self.phi1)*np.cos(self.theta1), -self.l1*np.cos(self.phi1)*np.sin(self.theta1), -self.l1*np.sin(self.phi1)*np.cos(self.theta1)],
		# 				  [  np.cos(self.phi1)*np.sin(self.theta1),  self.l1*np.cos(self.phi1)*np.cos(self.theta1), -self.l1*np.sin(self.phi1)*np.sin(self.theta1)], 
		# 				  [  np.sin(self.phi1)*np.cos(self.theta1), -self.l1*np.sin(self.phi1)*np.sin(self.theta1),  self.l1*np.cos(self.phi1)*np.cos(self.theta1)]])
		# self.J2=np.array([[  np.cos(self.phi2)*np.cos(self.theta2), -self.l2*np.cos(self.phi2)*np.sin(self.theta2), -self.l2*np.sin(self.phi2)*np.cos(self.theta2)], 
		# 				  [  np.cos(self.phi2)*np.sin(self.theta2),  self.l2*np.cos(self.phi2)*np.cos(self.theta2), -self.l2*np.sin(self.phi2)*np.sin(self.theta2)], 
		# 				  [  np.sin(self.phi2)*np.cos(self.theta2), -self.l2*np.sin(self.phi2)*np.sin(self.theta2),  self.l2*np.cos(self.phi2)*np.cos(self.theta2)]])
		# self.J3=np.array([[  np.cos(self.phi3)*np.cos(self.theta3), -self.l3*np.cos(self.phi3)*np.sin(self.theta3), -self.l3*np.sin(self.phi3)*np.cos(self.theta3)], 
		# 				  [  np.cos(self.phi3)*np.sin(self.theta3),  self.l3*np.cos(self.phi3)*np.cos(self.theta3), -self.l3*np.sin(self.phi3)*np.sin(self.theta3)], 
		# 				  [  np.sin(self.phi3)*np.cos(self.theta3), -self.l3*np.sin(self.phi3)*np.sin(self.theta3),  self.l3*np.cos(self.phi3)*np.cos(self.theta3)]])
		# self.J0=np.array([[  np.cos(self.phi0)*np.cos(self.theta0), -self.l0*np.cos(self.phi0)*np.sin(self.theta0), -self.l0*np.sin(self.phi0)*np.cos(self.theta0)],
		# 				  [  np.cos(self.phi0)*np.sin(self.theta0),  self.l0*np.cos(self.phi0)*np.cos(self.theta0), -self.l0*np.sin(self.phi0)*np.sin(self.theta0)],
		# 				  [  np.sin(self.phi0)*np.cos(self.theta0), -self.l0*np.sin(self.phi0)*np.sin(self.theta0),  self.l0*np.cos(self.phi0)*np.cos(self.theta0)]])
		# self.J1=np.array([[  np.cos(self.phi1)*np.cos(self.theta1), -self.l1*np.cos(self.phi1)*np.sin(self.theta1), -self.l1*np.sin(self.phi1)*np.cos(self.theta1)],
		# 				  [ -np.cos(self.phi1)*np.sin(self.theta1), -self.l1*np.cos(self.phi1)*np.cos(self.theta1),  self.l1*np.sin(self.phi1)*np.sin(self.theta1)], 
		# 				  [  np.sin(self.phi1)*np.cos(self.theta1), -self.l1*np.sin(self.phi1)*np.sin(self.theta1),  self.l1*np.cos(self.phi1)*np.cos(self.theta1)]])
		# self.J2=np.array([[ -np.cos(self.phi2)*np.cos(self.theta2),  self.l2*np.cos(self.phi2)*np.sin(self.theta2),  self.l2*np.sin(self.phi2)*np.cos(self.theta2)], 
		# 				  [ -np.cos(self.phi2)*np.sin(self.theta2), -self.l2*np.cos(self.phi2)*np.cos(self.theta2),  self.l2*np.sin(self.phi2)*np.sin(self.theta2)], 
		# 				  [  np.sin(self.phi2)*np.cos(self.theta2), -self.l2*np.sin(self.phi2)*np.sin(self.theta2),  self.l2*np.cos(self.phi2)*np.cos(self.theta2)]])
		# self.J3=np.array([[ -np.cos(self.phi3)*np.cos(self.theta3),  self.l3*np.cos(self.phi3)*np.sin(self.theta3),  self.l3*np.sin(self.phi3)*np.cos(self.theta3)], 
		# 				  [  np.cos(self.phi3)*np.sin(self.theta3),  self.l3*np.cos(self.phi3)*np.cos(self.theta3), -self.l3*np.sin(self.phi3)*np.sin(self.theta3)], 
		# 				  [  np.sin(self.phi3)*np.cos(self.theta3), -self.l3*np.sin(self.phi3)*np.sin(self.theta3),  self.l3*np.cos(self.phi3)*np.cos(self.theta3)]])
		
		self.J0=np.array([[  np.cos(self.phi0)*np.cos(self.theta0), -self.l0*np.cos(self.phi0)*np.sin(self.theta0), -self.l0*np.sin(self.phi0)*np.cos(self.theta0)],
						  [  np.cos(self.phi0)*np.sin(self.theta0),  self.l0*np.cos(self.phi0)*np.cos(self.theta0), -self.l0*np.sin(self.phi0)*np.sin(self.theta0)],
						  [  np.sin(self.phi0)                    ,  0                                            ,  self.l0*np.cos(self.phi0)                    ]])
		self.J1=np.array([[  np.cos(self.phi1)*np.cos(self.theta1), -self.l1*np.cos(self.phi1)*np.sin(self.theta1), -self.l1*np.sin(self.phi1)*np.cos(self.theta1)],
						  [  np.cos(self.phi1)*np.sin(self.theta1),  self.l1*np.cos(self.phi1)*np.cos(self.theta1), -self.l1*np.sin(self.phi1)*np.sin(self.theta1)], 
						  [  np.sin(self.phi1)                    ,  0                                            ,  self.l1*np.cos(self.phi1)                    ]])
		self.J2=np.array([[  np.cos(self.phi2)*np.cos(self.theta2), -self.l2*np.cos(self.phi2)*np.sin(self.theta2), -self.l2*np.sin(self.phi2)*np.cos(self.theta2)], 
						  [  np.cos(self.phi2)*np.sin(self.theta2),  self.l2*np.cos(self.phi2)*np.cos(self.theta2), -self.l2*np.sin(self.phi2)*np.sin(self.theta2)], 
						  [  np.sin(self.phi2)                    ,  0                                            ,  self.l2*np.cos(self.phi2)                    ]])
		self.J3=np.array([[  np.cos(self.phi3)*np.cos(self.theta3), -self.l3*np.cos(self.phi3)*np.sin(self.theta3), -self.l3*np.sin(self.phi3)*np.cos(self.theta3)], 
						  [  np.cos(self.phi3)*np.sin(self.theta3),  self.l3*np.cos(self.phi3)*np.cos(self.theta3), -self.l3*np.sin(self.phi3)*np.sin(self.theta3)], 
						  [  np.sin(self.phi3)                    ,  0                                            ,  self.l3*np.cos(self.phi3)                    ]])
		self.Ja0=np.transpose([self.J0[:,0]])
		self.Ja1=np.transpose([self.J1[:,0]])
		self.Ja2=np.transpose([self.J2[:,0]])
		self.Ja3=np.transpose([self.J3[:,0]])
		self.Jp0=self.J0[:,1:] 
		self.Jp1=self.J1[:,1:] 
		self.Jp2=self.J2[:,1:] 
		self.Jp3=self.J3[:,1:] 
		self.Ha_0=np.concatenate((self.Ja0,-self.Ja1, zero_col, zero_col),axis=1)
		self.Ha_1=np.concatenate((zero_col,-self.Ja1, self.Ja2, zero_col),axis=1)
		self.Ha_2=np.concatenate((zero_col, zero_col, self.Ja2,-self.Ja3),axis=1)
		self.Ha = np.concatenate((self.Ha_0, self.Ha_1, self.Ha_2),axis=0)
		self.Hp_0=np.concatenate((self.Jp0,-self.Jp1, zero_col, zero_col, zero_col, zero_col),axis=1)
		self.Hp_1=np.concatenate((zero_col, zero_col,-self.Jp1, self.Jp2, zero_col, zero_col),axis=1)
		self.Hp_2=np.concatenate((zero_col, zero_col, zero_col, zero_col, self.Jp2,-self.Jp3),axis=1)
		self.Hp = np.concatenate((self.Hp_0, self.Hp_1, self.Hp_2),axis=0)


		self.Hp_inv = np.linalg.pinv(self.Hp)
		self.G = np.matmul(-self.Hp_inv,self.Ha)

		self.temp0 = np.array([1,0,0,0])
		self.temp0 = np.concatenate(([self.temp0],[self.G[0]],[self.G[1]]),axis =0)
		print(self.temp0)
		self.temp0 = np.matmul(self.J0,self.temp0)

		self.temp1 = np.array([0,1,0,0])
		self.temp1 = np.concatenate(([self.temp1],[self.G[2]],[self.G[3]]),axis =0)
		# print(self.temp1)
		self.temp1 = np.matmul(self.J1,self.temp1)

		self.temp2 = np.array([0,0,1,0])
		self.temp2 = np.concatenate(([self.temp2],[self.G[4]],[self.G[5]]),axis =0)
		# print(self.temp2)
		self.temp2 = np.matmul(self.J2,self.temp2)

		self.temp3 = np.array([0,0,0,1])
		self.temp3 = np.concatenate(([self.temp3],[self.G[6]],[self.G[7]]),axis =0)
		# print(self.temp3)
		self.temp3 = np.matmul(self.J3,self.temp3)

		print("actuator jacobians")
		print(self.temp0)
		print(self.temp1)
		print(self.temp2)
		print(self.temp3)
		print("actuator jacobians multiplied test")
		print(np.matmul(self.temp0,np.array([[0.1311],[-0.0044],[-0.0809],[-0.06357]])))
		print(np.matmul(self.temp1,np.array([[0.3],[0],[-0.3],[0]])))
		print(np.matmul(self.temp2,np.array([[0.3],[0],[-0.3],[0]])))
		print(np.matmul(self.temp3,np.array([[0.3],[0],[-0.3],[0]])))
		print("conditions for actuator jacobians")
		print(np.linalg.cond(self.temp0))
		print(np.linalg.cond(self.temp1))
		print(np.linalg.cond(self.temp2))
		print(np.linalg.cond(self.temp3))
		self.temp_transpose0 = np.linalg.pinv(self.temp0)
		self.temp_transpose1 = np.linalg.pinv(self.temp1)
		self.temp_transpose2 = np.linalg.pinv(self.temp2)
		self.temp_transpose3 = np.linalg.pinv(self.temp3)
		print("conditions for inverted actuator jacobians")
		print(np.matmul(self.temp0,self.temp_transpose0))
		print(np.matmul(self.temp1,self.temp_transpose1))
		print(np.matmul(self.temp2,self.temp_transpose2))
		print(np.matmul(self.temp3,self.temp_transpose3))
		self.l0_dot = np.matmul(self.temp_transpose0, np.array([[self.x_dot],[self.y_dot],[self.z_dot]]))
		self.l1_dot = np.matmul(self.temp_transpose1, np.array([[self.x_dot],[self.y_dot],[self.z_dot]]))
		self.l2_dot = np.matmul(self.temp_transpose2, np.array([[self.x_dot],[self.y_dot],[self.z_dot]]))
		self.l3_dot = np.matmul(self.temp_transpose3, np.array([[self.x_dot],[self.y_dot],[self.z_dot]]))
		print("calculated cable speeds for given platform speed")
		print(self.l0_dot)
		print(self.l1_dot)
		print(self.l2_dot)
		print(self.l3_dot)



	def update_q():
		pass

obj = Kinematics()
obj.forward_kinematics()
print("_________________________________")
obj.get_Jacobians()
# x=0.5
# y=0.5
# z=0.5
# theta = np.arctan2(y,x)
# phi = np.arctan2(z,x)
# l1 = (x)/(np.cos(theta)*np.cos(phi))
# print(l1)
# print(theta)
# print(phi)

