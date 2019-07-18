#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import h5py
from genesis_parking.msg import state_est
import pdb
import rospkg


class PlotGPSTrajectory():
	'''
	A class to plot the global GPS trajectory and the vehicle's path tracking behavior.
	'''
	def __init__(self):
		rospack         = rospkg.RosPack()
		base            = rospack.get_path('genesis_parking') + '/data/'
		jld_file         = 'parkPath.jld'
		
		path_dict = h5py.File(base + jld_file, 'r')
		
		# Set up Data
		self.x_global_traj = path_dict['sp10'].value[:,0]
		self.y_global_traj = path_dict['sp10'].value[:,1]
		self.x_obs = path_dict['ox'].value
		self.y_obs = path_dict['oy'].value
		
		self.x_vehicle = self.x_global_traj[0];  self.y_vehicle = self.y_global_traj[0];

		# Set up Plot
		self.f = plt.figure()		
		plt.ion()
		l1, = plt.plot(self.x_global_traj, self.y_global_traj, 'r') 			
		l2, = plt.plot(self.x_obs, self.y_obs, 'k.')
		l3, = plt.plot(self.x_vehicle, self.y_vehicle, 'bo')	

		plt.xlabel('X (m)'); plt.ylabel('Y (m)')
		self.l_arr = [l1,l2,l3]
		plt.axis('equal')
		
		rospy.init_node('vehicle_plotter', anonymous=True)
		rospy.Subscriber('state_estimate', state_est, self.update_state, queue_size=1)
		self.loop()

	def loop(self):
		# Main Plotting Loop.  Updates plot with info from subscriber callbacks.
		r  = rospy.Rate(10)
		while not rospy.is_shutdown():
			# Update Plot with fresh data.
			self.l_arr[2].set_xdata(self.x_vehicle);  self.l_arr[2].set_ydata(self.y_vehicle)		
			self.f.canvas.draw()
			plt.pause(0.001)
			r.sleep()

	def update_state(self, msg):
		# Update vehicle's position.
		self.x_vehicle = msg.x
		self.y_vehicle = msg.y
		
if __name__=='__main__':
	p = PlotGPSTrajectory()
