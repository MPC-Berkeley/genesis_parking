#!/usr/bin/env python
import rosbag
import rospkg
import matplotlib.pyplot as plt
from math import radians
import h5py


class RosbagLoader():
	def __init__(self):
		self.rospack		= rospkg.RosPack()
		self.base			= self.rospack.get_path('genesis_parking')
		self.bag_file		= '/launch/controllerTest_2018-09-28-23-46-25.bag'
		self.b				= rosbag.Bag(self.base + self.bag_file)
		self.jld_file		= '/data/parkPath.jld'
		self.path_dict		= h5py.File(self.base + self.jld_file, 'r')
		
		self.x_obs			= self.path_dict['ox'].value
		self.y_obs			= self.path_dict['oy'].value
		
		self.t_enable		= []
		self.enable			= []
		
		self.t_steer		= []
		self.steer_angle	= []
		
		self.t_actual_steer	= []
		self.actual_steer	= []
		
		self.t_acc			= []
		self.acc			= []
		
		self.t_actual_acc	= []
		self.actual_acc		= []
		
		self.t_x			= []
		self.x				= []
		
		self.t_y			= []
		self.y				= []
		
		for topic, msg, t in self.b.read_messages('/vehicle/spas_mode'): 
			self.t_enable.append(t.secs + t.nsecs*1e-9)
			self.enable.append(msg.data)
			if msg.data == True:
				self.t_cutoff = t.secs+t.nsecs*1e-9
				break
		
		for topic, msg, t in self.b.read_messages('/control/steer_angle'):
			self.t_steer.append(t.secs + t.nsecs*1e-9)
			self.steer_angle.append(msg.data)
			
		for topic, msg, t in self.b.read_messages('/vehicle/steering'):
			self.t_actual_steer.append(t.secs + t.nsecs*1e-9)
			self.actual_steer.append(radians(msg.steering_wheel_angle)/15.87)
			
		for topic, msg, t in self.b.read_messages('/control/accel'):
			self.t_acc.append(t.secs + t.nsecs*1e-9)
			self.acc.append(msg.data)
		
		#for topic, msg, t in self.b.read_messages('/vehicle/imu'):
		#	self.t_actual_acc.append(t.secs + t.nsecs*1e-9)
		#	self.actual_acc.append(msg.data)	
		
		for topic, msg, t in self.b.read_messages('/state_estimate'):
			self.t_x.append(t.secs + t.nsecs*1e-9)
			self.x.append(msg.x)
			self.t_y.append(t.secs + t.nsecs*1e-9)
			self.y.append(msg.y)
		
		plt.axvline(x=self.t_cutoff, label='control enabled', color='r')
		plt.plot(self.t_steer, self.steer_angle, label='steering angle control')
		plt.plot(self.t_actual_steer, self.actual_steer, label='actual steering angle')
		plt.legend()
		plt.show()
		
		plt.axvline(x=self.t_cutoff, label='control enabled', color='r')
		plt.plot(self.t_acc, self.acc, label='acceleration')
		#plt.plot(self.t_actual_acc, self.actual_acc, label='actual acceleration')
		plt.legend()
		plt.show()
		
		plt.plot(self.x, self.y, label='path')
		plt.plot(self.x_obs, self.y_obs, 'k.')
		plt.legend()
		plt.axis('equal')
		plt.xlabel('X (m)')
		plt.ylabel('Y (m)')
		plt.show()
		

if __name__=='__main__':
	r = RosbagLoader()
		
		

