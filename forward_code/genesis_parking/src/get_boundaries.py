#!/usr/bin/env python
import rospy
#import scipy.io as sio
#from gmplot import gmplot
import numpy as np
import operator
import pdb
from numpy import pi, sin, cos, array, radians, dot
import rospkg
import csv
import matplotlib.pyplot as plt

class CoordinateSystem():
    def __init__(self,latitude_reference,longitude_ref):
        self.R_earth    = 6371000 # meters
        self.lat_ref    = latitude_reference
        self.lng_ref    = longitude_ref


    def get_XY(self,lat, lng):
        # Sources: http://www.movable-type.co.uk/scripts/latlong.html
        delta_lat = radians(lat - self.lat_ref)
        delta_lon = radians(lng - self.lng_ref)

        lat_avg = 0.5 * ( radians(lat) + radians(self.lat_ref) )
        X = self.R_earth * delta_lon * cos(lat_avg)
        Y = self.R_earth * delta_lat

        return X,Y

def plotCar(state):
	Lr = 1.498
	Lf = 1.521
	Lbf = 0.845
	Lbr = 1.135
	wl = 1.89 / 2
	wr = 1.89 / 2
	x = state[0]
	y = state[1]
	yaw = state[2]
	rearLeft = [x- Lbr*cos(yaw) - wl * sin(yaw), y - Lbr*sin(yaw) + wl * cos(yaw)]
	rearRight = [rearLeft[0] + (wl + wr) * sin(yaw), rearLeft[1] - (wl + wr) * cos(yaw)]
	frontLeft = [rearLeft[0] + (Lr+Lf+Lbf+Lbr) * cos(yaw), rearLeft[1] + (Lr+Lf+Lbf+Lbr) * sin(yaw)]
	frontRight = [frontLeft[0] + (wl + wr) * sin(yaw), frontLeft[1] - (wl + wr) * cos(yaw)]
	boundLeft = [[rearLeft[0], frontLeft[0]], [rearLeft[1], frontLeft[1]]]
	boundFront = [[frontLeft[0], frontRight[0]], [frontLeft[1], frontRight[1]]]
	boundRight = [[rearRight[0], frontRight[0]], [rearRight[1], frontRight[1]]]
	boundRear = [[rearLeft[0], rearRight[0]], [rearLeft[1], rearRight[1]]]
	plt.plot(boundLeft[0], boundLeft[1], 'k', boundFront[0], boundFront[1], 'k', boundRight[0], boundRight[1], 'k', boundRear[0], boundRear[1], 'k')
	

def main():
    # initialize node
    rospy.init_node("boundary_generator", anonymous=True)

    # get parameters
    Lf          = rospy.get_param("/length_front")
    Lr          = rospy.get_param("/length_rear") 
    w           = rospy.get_param("/width") 
    w_margin    = rospy.get_param("/width_margin") 
    length_front_bumber   = rospy.get_param("/length_front_bumber")
    length_rear_bumber   = rospy.get_param("/length_rear_bumber")
    select      = rospy.get_param("/parking_lot")
    plot_gps    = rospy.get_param("/plot_gps")   
    GPS_com 	= rospy.get_param("/GPS_com2")
    GM_ref 		= rospy.get_param("/GM_ref2")
    L_axle_bumper = rospy.get_param("/length_rear_axle_to_bumper")
    spot_label  = rospy.get_param("/desired_spot")
    spot_margin	= rospy.get_param("/spot_margin")
    spotBuffer	= rospy.get_param("/spot_buffer")
    spot_dict   = rospy.get_param("/%s" % spot_label)

    # initialize data
    lng0, lat0, yaw0    = (0.0, 0.0, 0.0)
    lngF, latF, yawF    = (0.0, 0.0, 0.0)
    lat_ref, lng_ref    = (0.0, 0.0)
    
    offset				= (length_front_bumber + Lr)/2 - Lr
    #yawF				= 1.1888903267948967
    yawF				= 1.2328743267948965
    
    offsetX				= -offset*cos(yawF)
    offsetY				= -offset*sin(yawF)
    
    GM_ref[1]			= GM_ref[1] + 180.0/pi*offsetX/(6371000.0*cos(pi*GM_ref[0]/180.0))
    GM_ref[0]			= GM_ref[0] + 180.0/pi*offsetY/6371000.0
    
    diff = np.array(GPS_com) - np.array(GM_ref)
    
    print(diff)

    # parking lot area 
    if select == 1:
        b1 = np.array(rospy.get_param("/b1_1")) + diff
        b2 = np.array(rospy.get_param("/b1_2")) + diff
        b3 = np.array(rospy.get_param("/b1_3")) + diff
        b4 = np.array(rospy.get_param("/b1_4")) + diff
    if select == 2:
    	raise ValueError("Not implemented yet!")

    bpts = [b1,b2,b3,b4]

    # read data file
    rospack         = rospkg.RosPack()
    base            = rospack.get_path('genesis_parking') + '/data/'
    p0_file         = 'initial.csv'
    vehicle_file    = 'vehicle_data.npz'
    boundary_file   = 'boundary_data.npz'
    
    #vehicle_file    = 'vehicle_data_2.npz'
    #boundary_file   = 'boundary_data_2.npz'

    
    with open(base + p0_file) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        next(csv_reader)
        yaw0, lng0, lat0     = next(csv_reader)
        lng0 = float(lng0)
        lat0 = float(lat0)
        yaw0 = float(yaw0)
    
    
    # Position of Parking Spot Vertices.
    gps_fl = np.array(spot_dict['left_front']) + diff
    gps_fr = np.array(spot_dict['right_front']) + diff
    gps_rl = np.array(spot_dict['left_back']) + diff
    gps_rr = np.array(spot_dict['right_back']) + diff
        
    # Final Position Computed wrt Parking Spot.
    GM_gc_lat = 0.25 * (gps_fl[0] + gps_rr[0] + gps_fr[0] + gps_rl[0])    # roughly the center of the spot
    GM_gc_lng = 0.25 * (gps_fl[1] + gps_rr[1] + gps_fr[1] + gps_rl[1])    # roughly the center of the spot
    
    
    
    # convert from longitude / latitude coordinates to (x,y) coordinates
    coord_sys           = CoordinateSystem(GM_gc_lat, GM_gc_lng)
    (x0,y0)             = coord_sys.get_XY(lat0, lng0)
    (xRf, yRf)			= coord_sys.get_XY(gps_fr[0], gps_fr[1])
    (xRr, yRr)			= coord_sys.get_XY(gps_rr[0], gps_rr[1])
    # yawF 				= np.arctan2((yRf - yRr), (xRf - xRr))
    yawF				= 1.2328743267948965
    xF					= -(Lr+offset)*cos(yawF)
    yF					= -(Lr+offset)*sin(yawF)
    x0					= x0 - Lr*cos(yaw0)
    y0					= y0 - Lr*sin(yaw0)
    s0                  = array( [x0, y0, yaw0, 0] )
    sF                  = array( [xF, yF, yawF, 0] )
    sF					= sF - [spotBuffer * cos(sF[2]), spotBuffer * sin(sF[2]), 0, 0]
    
    
    #GM_gc_lng			= GM_gc_lng + 180.0/pi*(xF+2*offsetX)/(6371000.0*cos(pi*GM_gc_lat/180.0))
    #GM_gc_lat			= GM_gc_lat + 180.0/pi*(yF+2*offsetY)/6371000.0
    
    
    gps_coords			= array( [GM_gc_lat, GM_gc_lng] )
    np.savez(base + vehicle_file, initial_state=s0, \
                         final_state=sF, \
                         length_front=Lf, \
                         length_rear=Lr, \
                         width=w , \
                         final_gps=gps_coords)
    

    b_fl = coord_sys.get_XY(gps_fl[0], gps_fl[1]) # XY coord, front left
    b_fl = tuple(map(operator.add, b_fl, (-spot_margin*cos(np.pi/2-yawF), spot_margin*sin(np.pi/2-yawF))))
    
    b_fr = coord_sys.get_XY(gps_fr[0], gps_fr[1]) # XY coord, front right
    b_fr = tuple(map(operator.add, b_fr, (spot_margin*cos(np.pi/2-yawF), -spot_margin*sin(np.pi/2-yawF))))
    
    b_rl = coord_sys.get_XY(gps_rl[0], gps_rl[1]) # XY coord, rear left
    b_rl = tuple(map(operator.add, b_rl, (-spot_margin*cos(np.pi/2-yawF), spot_margin*sin(np.pi/2-yawF))))
    
    b_rr = coord_sys.get_XY(gps_rr[0], gps_rr[1]) # XY coord, rear right
    b_rr = tuple(map(operator.add, b_rr, (spot_margin*cos(np.pi/2-yawF), -spot_margin*sin(np.pi/2-yawF))))
    
    bpts_xy = [] # boundary corresponding to overall parking lot
    for bpt in bpts:
        (bx,by)             = coord_sys.get_XY(bpt[0], bpt[1])
        bpts_xy.append([bx,by])
	
    # build constraints
    # parking lot in front of California PATH building
    if select == 1:
        b1 = array([bpts_xy[1],
                    bpts_xy[0]]) 
        b2 = array([bpts_xy[2],
                    b_rr,
                    b_fr]) 
        b3 = array([b_fr, 
                    b_fl])
        b4 = array([b_fl, 
                    b_rl,
                    bpts_xy[3]])  

        np.savez(base + boundary_file, b1 = b1, \
                                       b2 = b2, \
                                       b3 = b3, \
                                       b4 = b4)
        plt.plot(s0[0], s0[1], 'go')
        plotCar(s0)
        plt.plot(sF[0], sF[1], 'ro')
        plotCar(sF)
        plt.plot([b1[0][0],b1[1][0]], [b1[0][1],b1[1][1]], 'k')
        plt.plot([b2[0][0],b2[1][0]], [b2[0][1],b2[1][1]], 'k')
        plt.plot([b2[1][0],b2[2][0]], [b2[1][1],b2[2][1]], 'k')
        plt.plot([b3[0][0],b3[1][0]], [b3[0][1],b3[1][1]], 'k')
        plt.plot([b4[0][0],b4[1][0]], [b4[0][1],b4[1][1]], 'k')
        plt.plot([b4[1][0],b4[2][0]], [b4[1][1],b4[2][1]], 'k')
        plt.axis('equal')
        plt.show()
        
    print "\n\nFinished generating vertices\n\n"


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
