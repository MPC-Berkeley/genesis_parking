#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg as la
from control import dare
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import UInt8
from std_msgs.msg import UInt32
import rospy
import h5py
from genesis_parking.msg import state_est
import pdb
import rospkg
import math


class LQR():

	def __init__(self):

		# Set up data

		self.sCurr = None
		self.gear = 1 			# 1 for forward, 14 for reverse
		self.gear_flag = False 	# Whether or not the gear needs to be changed
		self.gear_msg = None 	# msg data
		self.real_car = True 	# simulation or real car
		
	

		# set up geometry
		self.rospack     	= rospkg.RosPack()
		self.Lf				= 1.521
		self.Lr				= 1.498
		self.L 				= 1.521+1.498
		self.base            = self.rospack.get_path('genesis_parking') + '/data/'
		self.jld_file        = 'parkPath.jld'

		self.path_dict = h5py.File(self.base + self.jld_file, 'r')
		
		# state: x, y, psi, v
		# input: theta, a
		self.s_global_traj = self.path_dict['sp10Ref'].value
		self.x_global_traj = self.path_dict['sp10Ref'].value[:,0]
		self.y_global_traj = self.path_dict['sp10Ref'].value[:,1]
		self.psi_global_traj = self.path_dict['sp10Ref'].value[:,2]
		self.v_global_traj = self.path_dict['sp10Ref'].value[:,3]
		
		self.u_global_traj = self.path_dict['up10Ref'].value
		self.theta_global_traj = self.path_dict['up10Ref'].value[:,0]
		self.acc_global_traj = self.path_dict['up10Ref'].value[:,1]



		# parse the path into several segments
		self.reverse_step = [0]
		
		for index in range(len(self.v_global_traj)-1):
			if round(self.v_global_traj[index], 5)*round(self.v_global_traj[index+1], 5) < 0:
				self.reverse_step.append(index+1)
				
		self.reverse_step.append(-1)
		
		
		# set up ros nodes
		rospy.init_node('LQR_controller', anonymous=True)
		self.state_sub = rospy.Subscriber('state_estimate', state_est, self.x_update, queue_size=1)
		self.gear_sub		= rospy.Subscriber('/vehicle/gear', UInt8, self.gear_update, queue_size=1)
		self.gear_change_pub	= rospy.Publisher('/gear_change', UInt8, queue_size=1) #1 for forward, 14 for reverse
		self.gear_flag_pub	= rospy.Publisher('/gear_flag', Bool, queue_size=1)
		self.acc_pub 		= rospy.Publisher('/control/accel', Float32, queue_size=1)
		self.acc_enable_pub 		= rospy.Publisher('/control/enable_accel', UInt8, queue_size=1)
		self.steer_pub 		= rospy.Publisher('/control/steer_angle', Float32, queue_size=1)
		self.ref_pub 		= rospy.Publisher('/state_ref', UInt32, queue_size=1)
		self.solved_pub 	= rospy.Publisher('/solveAll', Bool, queue_size=1)
		
		self.ts = 0.01
	
		self.r = rospy.Rate(int(1/self.ts))
		
		# lqr weights
		Q_f = np.matrix([ \
		[60.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], \
		[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], \
		[0.0, 0.0, 80.0, 0.0, 0.0, 0.0, 0.0], \
		[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], \
		[0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0], \
		[0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0], \
		[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
		R_f = np.matrix([[30.0, 0.0], [0.0, 1.0]])
		
		
		
		Q_f_decoupled = np.matrix([ \
		[60.0, 0.0, 0.0, 0.0, 0.0], \
		[0.0, 0.0, 0.0, 0.0, 0.0], \
		[0.0, 0.0, 80.0, 0.0, 0.0], \
		[0.0, 0.0, 0.0, 0.0, 0.0], \
		[0.0, 0.0, 0.0, 0.0, 1.0]])
		R_f_decoupled = np.matrix([[30.0]])
		
		Q_b = np.matrix([ \
		[100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], \
		[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], \
		[0.0, 0.0, 60.0, 0.0, 0.0, 0.0, 0.0], \
		[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], \
		[0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0], \
		[0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0], \
		[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
		R_b = np.matrix([[100.0, 0.0], [0.0, 0.0]])
		
		
		
		Q_b_decoupled = np.matrix([ \
		[100.0, 0.0, 0.0, 0.0, 0.0], \
		[0.0, 0.0, 0.0, 0.0, 0.0], \
		[0.0, 0.0, 60.0, 0.0, 0.0], \
		[0.0, 0.0, 0.0, 0.0, 0.0], \
		[0.0, 0.0, 0.0, 0.0, 1.0]])
		R_b_decoupled = np.matrix([[100.0]])
		
		# tolerence for final states
		tol = (1.20, 0.95, 0.50)
		
		# for each segment of the path, set up correct weights and gear, then call the lqr controller
		for index in range(len(self.reverse_step)-1):
			self.final_distance = 100 # placeholder (will be recalculated later)
			self.xref_temp = self.s_global_traj[self.reverse_step[index]:self.reverse_step[index+1], :]
			self.uref_temp = self.u_global_traj[self.reverse_step[index]:self.reverse_step[index+1], :]
			
			self.s_index_temp = self.reverse_step[index]
			
			if round(self.v_global_traj[self.reverse_step[index]], 5) < 0:
				Q_temp = Q_b
				R_temp = R_b
				Q_decoupled_temp = Q_b_decoupled
				R_decoupled_temp = R_b_decoupled
				if self.gear == 14:
					self.gear_flag = True
				else:
					self.gear_flag = False
					self.gear_msg = 14
			else:
				Q_temp = Q_f
				R_temp = R_f
				Q_decoupled_temp = Q_f_decoupled
				R_decoupled_temp = R_f_decoupled
				if self.gear == 1:
					self.gear_flag = True
				else:
					self.gear_flag = False
					self.gear_msg = 1
				
			
			self.gear_flag_pub.publish(self.gear_flag)
			if self.gear_flag == False:
				print("Please shift gear")
				self.steer_pub.publish(0.0)
				if self.real_car == True:
					self.acc_pub.publish(-3.0) #for real car
				else:
					self.acc_pub.publish(0.0) # for simulator
				self.gear_change_pub.publish(self.gear_msg)
					
			while self.gear_flag == False:
				if self.real_car == True:
					self.acc_pub.publish(-3.0) #for real car
				else:
					self.acc_pub.publish(0.0) # for simulator
					
				rospy.sleep(0.5)
				if self.gear == self.gear_msg:
					self.gear_flag = True
					self.gear_flag_pub.publish(self.gear_flag)
				
			if self.gear_flag == True:
				print("Gear in correct position")
				rospy.sleep(2)
				
			flag = self.lqr_ref_tracking(self.xref_temp, self.uref_temp, Q_temp, R_temp, Q_decoupled_temp, R_decoupled_temp, tol[index])
			print("One Segment Finished")
			
		self.solved_pub.publish(True)
		self.acc_enable_pub.publish(1)
		self.steer_pub.publish(0.0)
		self.acc_pub.publish(-3.0)
		print("All Segments Finished")
			


	def calculate_A(self, current):
		
		
		#A = np.matrix([[1.0, self.ts, 0.0, 0.0, 0.0], [0.0, 0.0, self.sCurr[3], 0.0, 0.0], [0.0, 0.0, 1.0, self.ts, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 1.0]])
		
		A = np.matrix([ \
		[1.0, self.ts, 0.0, 0.0, 0.0, 0.0, 0.0], \
		[0.0, 0.0, current[3], 0.0, 0.0, 0.0, 0.0], \
		[0.0, 0.0, 1.0, self.ts, 0.0, 0.0, 0.0], \
		[0.0, 0.0, 0.0, 0.0, 0.0, current[3]/self.L, 0.0], \
		[0.0, 0.0, 0.0, 0.0, 1.0, 0.0, self.ts], \
		[0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0], \
		[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
		
		
		return A
		
	def calculate_decoupled_A(self, current):
		
		#A = np.matrix([[1.0, self.ts, 0.0, 0.0], [0.0, 0.0, self.sCurr[3], 0.0], [0.0, 0.0, 1.0, self.ts], [0.0, 0.0, 0.0, 0.0]])
		
		A = np.matrix([ \
		[1.0, self.ts, 0.0, 0.0, 0.0], \
		[0.0, 0.0, current[3], 0.0, 0.0], \
		[0.0, 0.0, 1.0, self.ts, 0.0], \
		[0.0, 0.0, 0.0, 0.0, current[3]/self.L], \
		[0.0, 0.0, 0.0, 0.0, 1.0]])
		
		return A

	def calculate_B(self):
		
		#B = np.matrix([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [self.sCurr[3]/self.L, 0.0], [0.0, self.ts]])
		
		B = np.matrix([ \
		[0.0, 0.0], \
		[0.0, 0.0], \
		[0.0, 0.0], \
		[0.0, 0.0], \
		[0.0, 0.0], \
		[1.0, 0.0], \
		[0.0, 1.0]])
		
		
		return B
		
	def calculate_decoupled_B(self):
		
		#B = np.matrix([[0.0], [0.0], [0.0], [self.sCurr[3]/self.L]])
		
		B = np.matrix([[0.0], [0.0], [0.0], [0.0], [1.0]])
		
		
		return B

	def x_update(self, msg):
		if self.gear != 14:
			self.sCurr = np.array([msg.x, msg.y, msg.psi, msg.v])
		elif self.gear == 14:
			self.sCurr = np.array([msg.x, msg.y, msg.psi, -msg.v])
		
	def gear_update(self, msg):
		self.gear = msg.data
		
	def mod2pi(self, angle):
		return (angle+math.pi)%(2*math.pi)-math.pi
		
	def modpi(self, angle):
		return (angle+math.pi/2)%(math.pi)-math.pi/2
		
	def calculate_distance(self, s1, s2):
		dx = s2[0]-s1[0]
		dy = s2[1]-s1[1]
		dpsi = s2[2]-s1[2]
		distance = math.sqrt(dx*dx+dy*dy+100*dpsi*dpsi)
		return distance
		
	def calculate_nominal_distance(self, s1, s2):
		dx = s1[0] - s2[0]
		dy = s1[1] - s2[1]
		distance = math.sqrt(dx*dx + dy*dy)
		if self.gear == 1:
			angle = self.mod2pi(s2[2]-math.atan2(dx,dy))
			distance = -math.sin(angle)*distance
		else:
			angle = self.mod2pi(s2[2]-math.atan2(dx,dy))
			distance = math.sin(angle)*distance
		
		return distance
		
	def find_min_distance(self, xref, current):
		dx = [current[0]-tempx for tempx in xref[:, 0]]
		dy = [current[1]-tempy for tempy in xref[:, 1]]
		dx2 = [x*x for x in dx]
		dy2 = [x*x for x in dy]
		distance2 = [a+b for a, b in zip(dx2, dy2)]
		index = distance2.index(min(distance2))
		
		return index
		
	def solve_dare(self, A, B, Q, R):
		P = Q
		error = 0.01
		max_iter = 150
		for i in range(max_iter):
			P_temp = A.T*P*A-A.T*P*B*la.inv(B.T*P*B+R)*B.T*P*A+Q
			if abs(P_temp-P).max()<error:
				break
			P = P_temp
		
		return P


	def lqr_ref_tracking(self, xref, uref, Q, R, Q_decoupled, R_decoupled, d):
	
		length = len(xref)
		
		
		
		p_ey = 0.0
		p_epsi = 0.0
		s_previous = self.sCurr
		s_ref_previous = self.sCurr
		u_ref_previous = np.array([0.0, 0.0])
		p_u = np.matrix([[0.0], [0.0]])
		u = np.matrix([[0.0], [0.0]])
		
		
		u_decoupled_ref_previous = 0.0
		p_u_decoupled = 0.0
		
		
		while self.sCurr is None:
			rospy.sleep(0.5)
		
		while (not rospy.is_shutdown()) and (self.final_distance > d):
			current = self.sCurr
			self.final_distance = self.calculate_distance(current, xref[-1, :])
			
			index = self.find_min_distance(xref, current)
			
			s_index = self.s_index_temp+index
			self.ref_pub.publish(s_index)
			
			# Coupled
			
			xLocalRef = xref[index, :]
			uLocalRef = uref[index, :]
			u_new_ref = np.matrix([uLocalRef[0] - u_ref_previous[0], uLocalRef[1] - u_ref_previous[1]])
			eyCurr = self.calculate_nominal_distance(current, xLocalRef)
			epsiCurr = current[2] - xLocalRef[2]
			eV = current[3] - xLocalRef[3]
			xCurrent = np.matrix([[eyCurr], [(eyCurr-p_ey)/self.ts], [epsiCurr], [(epsiCurr-p_epsi)/self.ts], [eV], [p_u[0]], [p_u[1]]])
			A = self.calculate_A(current)
			B = self.calculate_B()
			P = self.solve_dare(A, B, Q, R)
			K = np.matrix(la.inv(B.T * P * B + R) * (B.T * P * A))
			u_dot = u_new_ref.T - K * np.matrix(xCurrent)
			
			
			u[0] = p_u[0]+self.mod2pi(u_dot[0])
			u[1] = p_u[1]+u_dot[1]
			
			# Decoupled
			eyCurr = self.calculate_nominal_distance(current, xLocalRef)
			epsiCurr = current[2] - xLocalRef[2]
			
			xDecoupledCurrent = np.matrix([[eyCurr], [(eyCurr-p_ey)/self.ts], [epsiCurr], [(epsiCurr-p_epsi)/self.ts], [p_u_decoupled]])
			u_dot_decoupled_ref = uref[index,0] - u_decoupled_ref_previous
			uDecoupledRef = np.matrix(u_dot_decoupled_ref)
			
			A_decoupled = self.calculate_decoupled_A(current)
			B_decoupled = self.calculate_decoupled_B()
			
			P_decoupled = self.solve_dare(A_decoupled, B_decoupled, Q_decoupled, R_decoupled)
			K_decoupled = np.matrix(la.inv(B_decoupled.T * P_decoupled * B_decoupled + R_decoupled) * (B_decoupled.T * P_decoupled * A_decoupled))
			
			u_dot_decoupled = np.matrix(uDecoupledRef).T - K_decoupled * np.matrix(xDecoupledCurrent)
			
			u_decoupled = p_u_decoupled + self.mod2pi(u_dot_decoupled)
			
			
			print(self.final_distance)
			
			#publish u
			if self.gear == 1:
				if u[1] > 2.0:
					self.acc_pub.publish(2.0)
					print("Acc: " + str(2.0))
				elif u[1] < -2.0:
					self.acc_pub.publish(-2.0)
					print("Acc: " + str(-2.0))
				else:
					self.acc_pub.publish(u[1])
					print("Acc: " + str(round(u[1],3)))
					
				if round(self.modpi(u_decoupled),4) > 0.45:
					self.steer_pub.publish(0.45)
					p_u_decoupled = 0.45
					print("Steering: " + str(0.45))
				elif round(self.modpi(u_decoupled),4) < -0.45:
					self.steer_pub.publish(-0.45)
					p_u_decoupled = -0.45
					print("Steering: " + str(-0.45))
				else:
					self.steer_pub.publish(self.modpi(u_decoupled))
					p_u_decoupled = self.modpi(u_decoupled)
					print("Steering: " + str(round(self.modpi(u_decoupled),3)))
			
			elif self.gear == 14:
				if self.real_car == True:
					'''
					User control longitudinal acceleration in reverse gear

					'''
				else:
				
					if u[1][0] > 2.0:							# for simulator
						self.acc_pub.publish(2.0)				# for simulator
						print("Acc: " + str(2.0))
					elif u[1][0] < -2.0:						# for simulator
						self.acc_pub.publish(-2.0)				# for simulator
						print("Acc: " + str(-2.0))
					else:										# for simulator
						self.acc_pub.publish(u[1])				# for simulator
						print("Acc: " + str(round(u[1],3)))
				
				if round(self.modpi(u_decoupled),4) > 0.45:
					self.steer_pub.publish(0.45)
					p_u_decoupled = 0.45
					print("Steering: " + str(0.45))
				elif round(self.modpi(u_decoupled),4) < -0.45:
					self.steer_pub.publish(-0.45)
					p_u_decoupled = -0.45
					print("Steering: " + str(-0.45))
				else:
					self.steer_pub.publish(self.modpi(u_decoupled))
					p_u_decoupled = self.modpi(u_decoupled)
					print("Steering: " + str(round(self.modpi(u_decoupled),3)))
			
			s_previous = current
			s_ref_previous = xLocalRef
			u_ref_previous[0] = uLocalRef[0]
			u_ref_previous[1] = uLocalRef[1]
			
			p_ey = eyCurr
			p_epsi = epsiCurr
			
			p_u[0] = p_u_decoupled
			p_u[1] = u[1]
			
			u_decoupled_ref_previous = uref[index,0]
			
			
			self.r.sleep()
			
		if self.real_car == True:
			self.acc_enable_pub.publish(1)						# for real car
			self.acc_pub.publish(-3.0)							# for real car
		
		return True





if __name__ == '__main__':
    print("Start")
    lqr = LQR()
    print("Done")
