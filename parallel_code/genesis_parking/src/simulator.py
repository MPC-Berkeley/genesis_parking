#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import h5py
from genesis_parking.msg import state_est
from genesis_parking.msg import mpcSol
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import UInt8
from std_msgs.msg import UInt32
from std_msgs.msg import Float32
import pdb
import rospkg
from numpy import pi, sin, cos, array, radians, dot

class PlotGPSTrajectory():
	'''
	A class to plot the global GPS trajectory and the vehicle's path tracking behavior.
	'''
	def __init__(self):		

		# Load Global Trajectory
		rospack         = rospkg.RosPack()
		base            = rospack.get_path('genesis_parking') + '/data/'
		jld_file         = 'parkPath.jld'
		
		path_dict = h5py.File(base + jld_file, 'r')
		
		# Set up Data
		self.x_global_traj = path_dict['sp10Ref'].value[:,0]
		self.y_global_traj = path_dict['sp10Ref'].value[:,1]
		self.psi_global_traj = path_dict['sp10Ref'].value[:,2]
		self.v_global_traj = path_dict['sp10Ref'].value[:,3]
		self.x_obs = path_dict['ox'].value
		self.y_obs = path_dict['oy'].value
		self.ts = path_dict['time_step'].value
		
		self.u_global_traj = path_dict['up10Ref'].value
		self.theta_global_traj = path_dict['up10Ref'].value[:,0]
		self.acc_global_traj = path_dict['up10Ref'].value[:,1]

		#self.x_ref_traj = self.x_global_traj[0]; self.y_ref_traj = self.y_global_traj[0]
		self.x_mpc_traj = self.x_global_traj; self.y_mpc_traj = self.y_global_traj
		self.x_vehicle = self.x_global_traj[0];  self.y_vehicle = self.y_global_traj[0]; self.psi_vehicle = self.psi_global_traj[0]
		
		self.s = [0]
		self.s_curr = 0
		self.s_index = 0
		self.v_vehicle = 0
		self.steering_angle = 0
		s_temp = 0
		x_prev = self.x_global_traj[0]
		y_prev = self.y_global_traj[0]
		self.reverse_step = [0]
		for index in range(len(self.v_global_traj)-1):
			if round(self.v_global_traj[index], 5)*round(self.v_global_traj[index+1], 5) < 0:
				self.reverse_step.append(index+1)
		self.reverse_step.append(-1)
		
		
		for index in range(len(self.reverse_step)-1):
			xref_temp = self.x_global_traj[self.reverse_step[index]:self.reverse_step[index+1]]
			yref_temp = self.y_global_traj[self.reverse_step[index]:self.reverse_step[index+1]]
			for i in range(len(xref_temp)):
				x_curr = xref_temp[i]
				y_curr = yref_temp[i]
				s_temp = s_temp + ((x_curr-x_prev)**2+(y_curr-y_prev)**2)**0.5
				self.s.append(s_temp)
				x_prev = x_curr
				y_prev = y_curr
				
				
		
		
		self.gear_flag = True
		self.gear = 11
		
		
		
		Lr = 1.498
		Lf = 1.521
		Lbf = 0.845
		Lbr = 1.135
		wl = 1.89 / 2
		wr = 1.89 / 2
		rearLeft = [self.x_vehicle- Lbr*cos(self.psi_vehicle) - wl * sin(self.psi_vehicle), self.y_vehicle - Lbr*sin(self.psi_vehicle) + wl * cos(self.psi_vehicle)]
		rearRight = [rearLeft[0] + (wl + wr) * sin(self.psi_vehicle), rearLeft[1] - (wl + wr) * cos(self.psi_vehicle)]
		frontLeft = [rearLeft[0] + (Lr+Lf+Lbf+Lbr) * cos(self.psi_vehicle), rearLeft[1] + (Lr+Lf+Lbf+Lbr) * sin(self.psi_vehicle)]
		frontRight = [frontLeft[0] + (wl + wr) * sin(self.psi_vehicle), frontLeft[1] - (wl + wr) * cos(self.psi_vehicle)]
		boundLeft = [[rearLeft[0], frontLeft[0]], [rearLeft[1], frontLeft[1]]]
		boundFront = [[frontLeft[0], frontRight[0]], [frontLeft[1], frontRight[1]]]
		boundRight = [[rearRight[0], frontRight[0]], [rearRight[1], frontRight[1]]]
		boundRear = [[rearLeft[0], rearRight[0]], [rearLeft[1], rearRight[1]]]


		# Set up Plot: includes full ("global") trajectory, target trajectory, MPC prediction trajectory, and vehicle position.
		self.f = plt.figure(figsize=(16, 7))		
		plt.ion()
		self.sub1 = self.f.add_subplot(121, aspect='equal')
		self.sub2 = self.f.add_subplot(1, 2, 2)
		self.txt = self.sub1.text(-2.7, 13.0, "Please Switch Gear", verticalalignment='top', fontsize=30, color='r')
		self.txt.set_visible(False)
		l1, = self.sub1.plot(self.x_global_traj, self.y_global_traj, 'k') 			
		#l2, = plt.plot(self.x_ref_traj,    self.y_ref_traj, 'rx')	
		l3, = self.sub1.plot(self.x_vehicle, self.y_vehicle, 'bo')	
		#l4, = plt.plot(self.x_mpc_traj, self.y_mpc_traj, 'r')
		l5, = self.sub1.plot(boundLeft[0], boundLeft[1], 'k')
		l6, = self.sub1.plot(boundFront[0], boundFront[1], 'k')
		l7, = self.sub1.plot(boundRight[0], boundRight[1], 'k')
		l8, = self.sub1.plot(boundRear[0], boundRear[1], 'k')
		l9, = self.sub1.plot(self.x_obs, self.y_obs, 'k.')
		l10, = self.sub2.plot(self.s, self.theta_global_traj, 'b')
		l11, = self.sub2.plot(self.s_curr, self.steering_angle, 'ro')

		self.sub1.set_xlabel('X (m)'); self.sub1.set_ylabel('Y (m)')
		self.sub2.set_xlabel('s (m)'); self.sub2.set_ylabel('v (m/s)')
		self.l_arr = [l1,l3,l5,l6,l7,l8,l9,l10,l11]
		#self.sub1.axis('equal')

		rospy.init_node('vehicle_plotter', anonymous=True)
		rospy.Subscriber('state_estimate', state_est, self.update_state, queue_size=1)
		rospy.Subscriber('/control/steer_angle', Float32, self.control_update, queue_size=1)
		rospy.Subscriber('/gear_flag', Bool, self.gear_flag_update, queue_size=1)
		rospy.Subscriber('/vehicle/gear', UInt8, self.gear_update, queue_size=1)
		rospy.Subscriber('/state_ref', UInt32, self.ref_update, queue_size=1)
		#rospy.Subscriber('/mpcSolution', mpcSol, self.update_mpc_trajectory, queue_size=1)
		self.loop()

	def loop(self):
		# Main Plotting Loop.  Updates plot with info from subscriber callbacks.
		r = rospy.Rate(int(1/self.ts))
		
		while not rospy.is_shutdown():
			if self.gear_flag == False:
				self.txt.set_visible(True)
			else:
				self.txt.set_visible(False)
				
			
			Lr = 1.498
			Lf = 1.521
			Lbf = 0.845
			Lbr = 1.135
			wl = 1.89 / 2
			wr = 1.89 / 2
			rearLeft = [self.x_vehicle- Lbr*cos(self.psi_vehicle) - wl * sin(self.psi_vehicle), self.y_vehicle - Lbr*sin(self.psi_vehicle) + wl * cos(self.psi_vehicle)]
			rearRight = [rearLeft[0] + (wl + wr) * sin(self.psi_vehicle), rearLeft[1] - (wl + wr) * cos(self.psi_vehicle)]
			frontLeft = [rearLeft[0] + (Lr+Lf+Lbf+Lbr) * cos(self.psi_vehicle), rearLeft[1] + (Lr+Lf+Lbf+Lbr) * sin(self.psi_vehicle)]
			frontRight = [frontLeft[0] + (wl + wr) * sin(self.psi_vehicle), frontLeft[1] - (wl + wr) * cos(self.psi_vehicle)]
			boundLeft = [[rearLeft[0], frontLeft[0]], [rearLeft[1], frontLeft[1]]]
			boundFront = [[frontLeft[0], frontRight[0]], [frontLeft[1], frontRight[1]]]
			boundRight = [[rearRight[0], frontRight[0]], [rearRight[1], frontRight[1]]]
			boundRear = [[rearLeft[0], rearRight[0]], [rearLeft[1], rearRight[1]]]
			
			
			
			
			# Update Plot with fresh data.
			#self.l_arr[1].set_xdata(self.x_ref_traj); self.l_arr[1].set_ydata(self.y_ref_traj)		
			self.l_arr[1].set_xdata(self.x_vehicle);  self.l_arr[1].set_ydata(self.y_vehicle)		
			#self.l_arr[2].set_xdata(self.x_mpc_traj); self.l_arr[2].set_ydata(self.y_mpc_traj)
			self.l_arr[2].set_xdata(boundLeft[0]);  self.l_arr[2].set_ydata(boundLeft[1])
			self.l_arr[3].set_xdata(boundFront[0]);  self.l_arr[3].set_ydata(boundFront[1])
			self.l_arr[4].set_xdata(boundRight[0]);  self.l_arr[4].set_ydata(boundRight[1])
			self.l_arr[5].set_xdata(boundRear[0]);  self.l_arr[5].set_ydata(boundRear[1])
			
			self.l_arr[8].set_xdata(self.s[self.s_index]);  self.l_arr[8].set_ydata(self.steering_angle)
			
			self.f.tight_layout()
			self.f.canvas.draw()
			plt.pause(0.001) #0.001
			r.sleep()
			
		
	

	def update_state(self, msg):
		# Update vehicle's position.
		self.x_vehicle = msg.x
		self.y_vehicle = msg.y
		self.psi_vehicle = msg.psi
		if self.gear == 1:
			self.v_vehicle = msg.v
		elif self.gear == 14:
			self.v_vehicle = -msg.v
			
	def control_update(self, msg):
		self.steering_angle = msg.data
			
	def ref_update(self, msg):
		# Update vehicle's position.
		self.s_index = msg.data

	def update_mpc_trajectory(self, msg):
		# Update the MPC planned (open-loop) trajectory.
		self.x_mpc_traj = msg.z1OL
		self.y_mpc_traj = msg.z2OL
		
	def gear_flag_update(self, msg):
		self.gear_flag = msg.data
		
	def gear_update(self, msg):
		self.gear = msg.data
		
		# Update the reference for the MPC module.
		#self.x_ref_traj = msg.xr
		#self.y_ref_traj = msg.yr


if __name__=='__main__':
	p = PlotGPSTrajectory()
