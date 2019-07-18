#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
import math as m
from genesis_parking.msg import state_est
from tf import transformations
from numpy import pi
from numpy import unwrap, array, load
from numpy import sin, cos, array, radians, dot
import h5py
import rospkg
import csv

# Vehicle State Publisher for the Hyundai Genesis.  Uses OxTS and vehicle CAN messages to localize.
class State():
	def __init__(self):
		# initialize position information
		self.yaw 			= 0
		self.longitude_ref 	= None
		self.latitude_ref 	= None
		self.longitude 		= None
		self.latitude 		= None
		self.x 				= None
		self.y 				= None
		self.x_off			= None
		self.y_off			= None
		self.axle_dist		= None
		self.psi 			= None
		self.v 				= None

		# initialize subscribers
		self.imu_sub 	= rospy.Subscriber("/imu/data", Imu, self.imu_callback, queue_size=1)
		self.gps_sub 	= rospy.Subscriber("/gps/fix", NavSatFix, self.gps_callback, queue_size=1)
		self.vel_sub 	= rospy.Subscriber('/gps/vel', TwistWithCovarianceStamped, self.vel_callback, queue_size=1)
		self.state_pub 	= rospy.Publisher("state_estimate", state_est, queue_size=1)

	def define_offsets(self, x_off, y_off, axle_dist):
		self.x_off = 0
		self.y_off = 0
		self.axle_dist = axle_dist
		
	def set_reference(self, lat,lng):
		self.latitude_ref 	= lat
		self.longitude_ref 	= lng

	def gps_callback(self,msg):
		self.longitude 	= msg.longitude
		self.latitude  	= msg.latitude
		if self.longitude_ref != None:
			self.compute_xy()
	
	def imu_callback(self,msg):
		ori 				= msg.orientation
		quat 				= (ori.x, ori.y, ori.z, ori.w)
		roll, pitch, yaw 	= transformations.euler_from_quaternion(quat)
		self.yaw         	= unwrap(array([self.yaw, yaw]), discont = pi)[1]
		self.psi 			= self.yaw + pi/2.0    # 90 deg rotation of gps coordinate frame

	def vel_callback(self,msg):
		v_east 		= msg.twist.twist.linear.x
		v_north 	= msg.twist.twist.linear.y
		v_gps 		= m.sqrt(v_east**2 + v_north**2)
		self.v 		= v_gps

	def compute_xy(self):
		R_earth 	= 6371000 # meters
		delta_lat 	= radians(self.latitude - self.latitude_ref)
		delta_lon 	= radians(self.longitude - self.longitude_ref)
		lat_avg 	= 0.5 * ( radians(self.latitude) + radians(self.latitude_ref) )
		self.x 		= R_earth * delta_lon * cos(lat_avg)# - cos(self.yaw) * 1.498
		self.y 		= R_earth * delta_lat #- sin(self.yaw * 1.498)

	def publish_state(self):
		if not None in (self.x, self.y, self.v, self.psi): 
			curr_state = state_est()
			curr_state.header.stamp = rospy.Time.now()
			#curr_state.x   = self.x + self.x_off
			#curr_state.y   = self.y + self.y_off
			curr_state.psi = self.psi 
			curr_state.v   = self.v
			curr_state.x_adj = self.x + self.x_off - self.axle_dist*cos(self.psi)
			curr_state.y_adj = self.y + self.y_off - self.axle_dist*sin(self.psi)
			curr_state.x   = curr_state.x_adj
			curr_state.y   = curr_state.y_adj

			self.state_pub.publish(curr_state)

def pub_loop():
	rospy.init_node('state_publisher', anonymous=True)	

	rospack         = rospkg.RosPack()

	base            = rospack.get_path('genesis_parking') + '/data/'
	

	pF_name = 'vehicle_data.npz'	
	pF_file = load(base+pF_name)
	lng_ref = float(pF_file['final_gps'][1])
	lat_ref = float(pF_file['final_gps'][0])
	
	dgps_offset = rospy.get_param("/dgps_global_offset")
	axle_dist   = rospy.get_param("/gps_to_rear_axle_dist")
	
	jld_file         = 'parkPath.jld'	
	path_dict = h5py.File(base + jld_file, 'r')
	# Set up Data
	ts = path_dict['time_step'].value
	
	state = State()
	state.set_reference(lat_ref, lng_ref)
	state.define_offsets(dgps_offset[0], dgps_offset[1], axle_dist)
	
	r = rospy.Rate(int(1/ts))

	while not rospy.is_shutdown():		
		state.publish_state()
		r.sleep()

if __name__=='__main__':
	try:
		pub_loop()
	except rospy.ROSInterruptException:
		pass
