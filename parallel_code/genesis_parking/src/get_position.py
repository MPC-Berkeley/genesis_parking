#!/usr/bin/env python

import numpy as np
from tf import transformations
from numpy import pi
from numpy import unwrap, array, cos
from sensor_msgs.msg import Imu, NavSatFix
import csv
import rospy
import rospkg

class Position():
	def __init__(self):
		# initialize position information
		self.yaw 			= 0
		self.longitude 		= 0
		self.latitude 		= 0

		# initialize subscribers
		self.imu_sub 	= rospy.Subscriber("/imu/data", Imu, self.imu_callback)
		self.gps_sub 	= rospy.Subscriber("/gps/fix", NavSatFix, self.gps_callback)
		self.type   	= rospy.get_param("/get_position/type")   # initial or final position


		# set data path
		self.filename 	= self.type + ".csv"
		rospack 		= rospkg.RosPack()
		self.base 		= rospack.get_path('genesis_parking') + '/data/'

		rospy.on_shutdown(self.save_position)
		rospy.spin()

	def imu_callback(self,msg):
		ori 				= msg.orientation
		quat 				= (ori.x, ori.y, ori.z, ori.w)
		roll, pitch, yaw 	= transformations.euler_from_quaternion(quat)
		self.yaw         	= unwrap(array([self.yaw, yaw]), discont = pi)[1]

	def gps_callback(self,msg):
		# update value
		self.longitude 	= msg.longitude
		self.latitude  	= msg.latitude

	def save_position(self):
		# data = [self.yaw + 0.5*pi, self.longitude + 180.0/pi*self.x0_offset/(6371000.0*cos(pi*self.latitude/180.0)), self.latitude + 180.0/pi*self.y0_offset/6371000.0]
		data = [self.yaw + 0.5*pi, self.longitude, self.latitude]



		with open(self.base + self.filename, 'w') as csvfile:
			file = csv.writer(csvfile, delimiter=',')
			file.writerow(["yaw", "longitude", "latitude"])
			file.writerow(data)


def main():
    
    rospy.init_node('positon', anonymous=True)
     
    try:
    	pos = Position()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
