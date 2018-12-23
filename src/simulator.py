#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import h5py
from genesis_parking.msg import state_est
from genesis_parking.msg import mpcSol
from genesis_parking.msg import right_path_boundary
from genesis_parking.msg import left_path_boundary
from std_msgs.msg import Float64MultiArray
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
		self.x_global_traj = path_dict['sp10'].value[:,0]
		self.y_global_traj = path_dict['sp10'].value[:,1]
		self.psi_global_traj = path_dict['sp10'].value[:,2]
		self.x_obs = path_dict['ox'].value
		self.y_obs = path_dict['oy'].value
		self.ts = path_dict['time_step'].value

		#self.x_ref_traj = self.x_global_traj[0]; self.y_ref_traj = self.y_global_traj[0]
		self.x_mpc_traj = self.x_global_traj; self.y_mpc_traj = self.y_global_traj
		self.x_vehicle = self.x_global_traj[0];  self.y_vehicle = self.y_global_traj[0]; self.psi_vehicle = self.psi_global_traj[0]
		

		self.left_path_boundary = [0, 0, 0, 0]
		self.right_path_boundary = [0, 0, 0, 0]
		
		self.left_path = [0]
		self.right_path = [0]
		
		self.left_x = [self.x_global_traj[0]]
		self.right_x = [self.x_global_traj[0]]
		
		self.left_y = [self.y_global_traj[0]]
		self.right_y = [self.y_global_traj[0]]
		
		
		
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
		self.f = plt.figure()		
		plt.ion()
		l1, = plt.plot(self.x_global_traj, self.y_global_traj, 'k') 			
		#l2, = plt.plot(self.x_ref_traj,    self.y_ref_traj, 'rx')	
		l3, = plt.plot(self.x_vehicle, self.y_vehicle, 'bo')	
		l4, = plt.plot(self.x_mpc_traj, self.y_mpc_traj, 'r')
		l5, = plt.plot(boundLeft[0], boundLeft[1], 'k')
		l6, = plt.plot(boundFront[0], boundFront[1], 'k')
		l7, = plt.plot(boundRight[0], boundRight[1], 'k')
		l8, = plt.plot(boundRear[0], boundRear[1], 'k')
		l9, = plt.plot(self.x_obs, self.y_obs, 'k.')
		l10, = plt.plot(self.left_x, self.left_y, 'b')
		l11, = plt.plot(self.right_x, self.right_y, 'b')

		plt.xlabel('X (m)'); plt.ylabel('Y (m)')
		self.l_arr = [l1,l3,l4,l5,l6,l7,l8,l9,l10,l11]
		plt.axis('equal')

		rospy.init_node('vehicle_plotter', anonymous=True)
		rospy.Subscriber('state_estimate', state_est, self.update_state, queue_size=1)
		rospy.Subscriber('/mpcSolution', mpcSol, self.update_mpc_trajectory, queue_size=1)
		rospy.Subscriber('left_path_boundary',Float64MultiArray, self.update_left_path_boundary, queue_size=1)
		rospy.Subscriber('right_path_boundary',Float64MultiArray, self.update_right_path_boundary, queue_size=1)
		self.loop()

	def loop(self):
		# Main Plotting Loop.  Updates plot with info from subscriber callbacks.
		r = rospy.Rate(int(1/self.ts))
		
		while not rospy.is_shutdown():
			Lr = 1.498
			Lf = 1.521
			Lbf = 0.5
			Lbr = 0.5
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
			
			self.left_path = np.poly1d(self.left_path_boundary)
			self.right_path = np.poly1d(self.right_path_boundary)
		
			self.left_x = np.linspace(-20, 20, 100)
			self.right_x = np.linspace(-20, 20, 100)
		
			self.left_y = self.left_path(self.left_x)
			self.right_y = self.right_path(self.right_x)
			
			# Update Plot with fresh data.
			#self.l_arr[1].set_xdata(self.x_ref_traj); self.l_arr[1].set_ydata(self.y_ref_traj)		
			self.l_arr[1].set_xdata(self.x_vehicle);  self.l_arr[1].set_ydata(self.y_vehicle)		
			self.l_arr[2].set_xdata(self.x_mpc_traj); self.l_arr[2].set_ydata(self.y_mpc_traj)
			self.l_arr[3].set_xdata(boundLeft[0]);  self.l_arr[3].set_ydata(boundLeft[1])
			self.l_arr[4].set_xdata(boundFront[0]);  self.l_arr[4].set_ydata(boundFront[1])
			self.l_arr[5].set_xdata(boundRight[0]);  self.l_arr[5].set_ydata(boundRight[1])
			self.l_arr[6].set_xdata(boundRear[0]);  self.l_arr[6].set_ydata(boundRear[1])
			self.l_arr[8].set_xdata(self.left_x);  self.l_arr[8].set_ydata(self.left_y)
			self.l_arr[9].set_xdata(self.right_x);  self.l_arr[9].set_ydata(self.right_y)
			
			self.f.canvas.draw()
			plt.pause(0.0001) #0.001
			r.sleep()
			
		
	

	def update_state(self, msg):
		# Update vehicle's position.
		self.x_vehicle = msg.x
		self.y_vehicle = msg.y
		self.psi_vehicle = msg.psi

	def update_mpc_trajectory(self, msg):
		# Update the MPC planned (open-loop) trajectory.
		self.x_mpc_traj = msg.z1OL
		self.y_mpc_traj = msg.z2OL
		
		# Update the reference for the MPC module.
		#self.x_ref_traj = msg.xr
		#self.y_ref_traj = msg.yr

	def update_left_path_boundary(self, msg):
		self.left_path_boundary = [msg.data[3], msg.data[2], msg.data[1], msg.data[0]]

	def update_right_path_boundary(self, msg):
		self.right_path_boundary = [msg.data[3], msg.data[2], msg.data[1], msg.data[0]]

if __name__=='__main__':
	p = PlotGPSTrajectory()
