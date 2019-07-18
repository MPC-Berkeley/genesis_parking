#!/usr/bin/env python
import rospy
import math as m
from genesis_parking.msg import state_est
from tf import transformations
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from numpy import unwrap, array, load
from numpy import sin, cos, tan, array, radians, dot
import rospkg
import csv
import h5py

# Vehicle State Publisher for the Hyundai Genesis.  Uses OxTS and vehicle CAN messages to localize.
class State():
	def __init__(self):
		# initialize position information
		rospack         = rospkg.RosPack()
		base            = rospack.get_path('genesis_parking') + '/data/'
		
		double_maneuver = False
		
		if double_maneuver == False:
			jld_file         = 'parkPath.jld'
		else:
			jld_file         = 'parkPath_2.jld'
		
		path_dict = h5py.File(base + jld_file, 'r')
		
		# Set up Data
		self.ts = path_dict['time_step'].value
		self.x_global_traj = path_dict['sp10'].value[:,0]
		self.y_global_traj = path_dict['sp10'].value[:,1]
		self.psi_global_traj = path_dict['sp10'].value[:,2]
		
		self.x 				= self.x_global_traj[0]
		self.y 				= self.y_global_traj[0]
		self.psi 			= self.psi_global_traj[0]
		self.v 				= 0
		self.solveMPC		= True
		self.flag			= False
		self.acc			= None
		self.steer			= None

		# initialize subscribers
		self.acc_sub 		= rospy.Subscriber("/control/accel", Float32, self.acc_callback, queue_size=2)
		self.steer_sub 		= rospy.Subscriber("/control/steer_angle", Float32, self.steer_callback, queue_size=2)
		self.state_pub	 	= rospy.Publisher("state_estimate", state_est, queue_size=1)
		self.solveMPC_sub	= rospy.Subscriber("/solveMPC", Bool, self.solveMPC_callback,queue_size=1)

	def steer_callback(self,msg):
		if not self.flag:
			self.steer = msg.data
	
	def acc_callback(self,msg):
		if not self.flag:
			self.acc = msg.data
		
	def solveMPC_callback(self,msg):
		self.solveMPC = msg.data
		
	def vehicle_model(self):
		if not self.solveMPC and self.v <= 0:
			self.flag = True 
		
		if not self.flag and not None in (self.acc, self.steer): 
			xOld		= self.x
			yOld		= self.y
			psiOld		= self.psi
			vOld		= self.v
			L			= 1.498 + 1.521
			self.x		= xOld + self.ts * vOld * cos(psiOld)
			self.y		= yOld + self.ts * vOld * sin(psiOld)
			self.psi	= psiOld + self.ts * vOld * tan(self.steer)/L
			self.v		= vOld + self.ts*self.acc
			if self.v < 0:
				self.v = 0
			
		if self.flag:
			self.v = 0
		

	def publish_state(self):
		if not None in (self.x, self.y, self.v, self.psi): 
			curr_state = state_est()
			curr_state.header.stamp = rospy.Time.now()
			curr_state.psi = self.psi 
			curr_state.v   = self.v
			curr_state.x_adj = self.x # + self.x_off - self.axle_dist*cos(self.psi)
			curr_state.y_adj = self.y # + self.y_off - self.axle_dist*sin(self.psi)
			curr_state.x   = curr_state.x_adj
			curr_state.y   = curr_state.y_adj

			self.state_pub.publish(curr_state)

def pub_loop():
	rospy.init_node('state_simulator', anonymous=True)	

	rospack         = rospkg.RosPack()

	base            = rospack.get_path('genesis_parking') + '/data/'
	
	double_maneuver = False
	
	if double_maneuver == False:
		jld_file         = 'parkPath.jld'
	else:
		jld_file         = 'parkPath_2.jld'
		
	path_dict = h5py.File(base + jld_file, 'r')
	# Set up Data
	ts = path_dict['time_step'].value
	
	#pF_file         = 'final.csv'
	#lng_ref, lat_ref  	= (None,None)
	#with open(base + pF_file) as csv_file:
	#	csv_reader = csv.reader(csv_file, delimiter=',')
	#	next(csv_reader)
	#	yawF, lngF, latF    = next(csv_reader)
	#	lng_ref 			= float(lngF)
	#	lat_ref 			= float(latF)
	
	r = rospy.Rate(int(1/ts))
	state = State()

	while not rospy.is_shutdown():		
		state.vehicle_model()
		state.publish_state()
		r.sleep()

if __name__=='__main__':
	try:
		pub_loop()
	except rospy.ROSInterruptException:
		pass
