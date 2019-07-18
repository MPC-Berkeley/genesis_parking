#!/usr/bin/env julia

###############
# H-OBCA: Hierarchical Optimization-based Collision Avoidance - a path planner for autonomous parking
# Copyright (C) 2018
# Alexander LINIGER [liniger@control.ee.ethz.ch; Automatic Control Lab, ETH Zurich]
# Xiaojing ZHANG [xiaojing.zhang@berkeley.edu; MPC Lab, UC Berkeley]
# Atsushi SAKAI [atsushisakai@global.komatsu; Komatsu Ltd / MPC Lab]
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
###############
# The paper describing the theory can be found here:
# 	X. Zhang, A. Liniger and F. Borrelli; "Optimization-Based Collision Avoidance"; Technical Report, 2017, [https://arxiv.org/abs/1711.03449]
#   X. Zhang, A. Liniger, A. Sakai and F. Borrelli; "Autonomous  Parking  using  Optimization-Based  Collision  Avoidance"; Technical Report, 2018 [add URL]
###############

###############
# Main file: computes Collision-Free and Minimum-Penetration trajectories for parking
#  1) The velocity of vehicly is roughly given by v ~ sqrt(2)*motionStep*sampleN / Ts * timeScale
# 	  to change velocity of vehicle, do one (or both) of the following:
#		a) increase the denominator y in "Ts = 0.6/y*sampleN " in get_path.jl  (default: y=3)
# 		b) lower the bounds of "timeScale" in ParkingSignedDist.jl  (default: [0.5, 1.2])
#		c) the velocity is roughly given by 
#  2) to change maximum accleration profile, change parameters in veloSmooth.jl and ParkingSignDist.jl
###############

using PyPlot
using CSV
using NPZ
using PyCall
using JLD
using Polynomials
import YAML

println("Loading modules ...")
include("setup.jl")
@pyimport scipy.interpolate as intp
@pyimport rospkg

println("Loading data ...")
rospack     		= rospkg.RosPack()
base_path      		= rospack[:get_path]("genesis_parking")
vehicle_data 		= npzread(base_path*"/data/vehicle_data.npz")
boundary_data 		= npzread(base_path*"/data/boundary_data.npz")
planning_data 		= YAML.load(open(base_path*"/config/path_planner.yaml"))

##################################################
# fixed or variable sampling time 1/0
fixTime 	= 0		# default: 0 (variable time steps)

#### problem parameters ####
TsPF 		= 0.05 							# time step for MPC-path following
sampleN 	= 3 							# number of samples (heuristic)
# sampleN 	= 6								############################################### test debugging
# Ts 			= 0.6/3*sampleN 				# time step for A* (heuristic)
Ts 			= 0.6/4*sampleN 				############################################### test debugging
motionStep 	= 0.1							# step length of Hybrid A*", must correspond to MOTION_RESOLUTION in hybrid_a_star.jl
# velocity is going to be roughly (sqrt(2)*sampleN*motionStep / Ts ) * timeScale
Lf 			= vehicle_data["length_front"]  # center of mass to front axle
Lr 			= vehicle_data["length_rear"]  	# center of mass to rear axle
Lf_bumber	= planning_data["length_front_bumber"]	# center of mass to front bumber
Lr_bumber	= planning_data["length_rear_bumber"]	# center of mass to rear bumber
wr 			= vehicle_data["width"]/2.0		# distance from center of rear axel to left edge of car
wl 			= vehicle_data["width"]/2.0		# distance from center of rear axel to right edge of car
L 			= Lf+Lr  						# wheel base (i.e., distance from rear axel to front axel) 
ego  		= [Lf_bumber,wr,Lr_bumber,wl,Lf,Lr] 				# vehicle dimensions
time_step 	= planning_data["time_step"]

# initial / final state
s0 			= vehicle_data["initial_state"]
sF 			= vehicle_data["final_state"]
XYbounds 	= [ -40 	,40    ,-40      ,40       ]

println("loading obstacle verticies")
obs1  = boundary_data["b1"]
obs2  = boundary_data["b2"]
obs3  = boundary_data["b3"]
obs4  = boundary_data["b4"]


lOb   = [ obs1, obs2, obs3, obs4 ]


# get metadata on obstacles
nOb 	= size(lOb,1)
vOb 	= zeros(nOb)
for i = 1:nOb
	vOb[i]  = size(lOb[i],1)
end
vObMPC 	= vOb-1		# adjustment for optimizaton problem
vOb 	= convert(Array{Int64,1},vOb)
vObMPC 	= convert(Array{Int64,1},vObMPC)

# build obstacles for Hybrid A* algorithm
ox = Float64[]
oy = Float64[]
 
println("Building obstacle vertices")
for i = 1:nOb 				# iterate through each obstacle
	for j = 1:vObMPC[i] 		# iterate through each vertex in obstacle
		dist 	= sqrt.(sum( (lOb[i][j+1,:] - lOb[i][j,:]).^2 ) )
		line 	= hcat(lOb[i][j,:],lOb[i][j+1,:])
		fintp 	= intp.interp1d([0 ,dist], line)
		points 	= fintp(0:0.1:dist)
		nPts 	= size(points,2)
		for k = 1:nPts
			push!(ox, Float64(points[1,k]))
		    push!(oy, Float64(points[2,k]))
		end
	end
end

# obtain H-representation of obstacles
println("computing H-representation for obstacle polytopes ....")
AOb, bOb = obstHrep(nOb, vOb, lOb)
i_cs 	= cumsum(vObMPC)

# as long as one false, then okay
for i = 1:nOb
	if i == 1
		i_0 = 1
	else
		i_0 = i_cs[i-1]+1
	end
	i_f = i_cs[i]

	AOb_i = AOb[i_0:i_f,:]
	bOb_i = bOb[i_0:i_f,:]

	#println("A", i,"=",AOb_i)
	#println("b", i,"=",bOb_i)
end	

# call Hybrid A*
tic()
println("calculating hybrid a* solution ...");
rx, ry, ryaw = hybrid_a_star.calc_hybrid_astar_path(s0[1], s0[2], s0[3], sF[1], sF[2], sF[3], ox, oy, hybrid_a_star.XY_GRID_RESOLUTION, hybrid_a_star.YAW_GRID_RESOLUTION, hybrid_a_star.OB_MAP_RESOLUTION)
timeHybAstar = toq();

println("smoothing hybrid a* solution ...")
xWS, uWS, N = getSmoothProfile(rx,ry,ryaw,Ts,sampleN,motionStep)
# println("===== size of xWS $(size(xWS))")
# for debugging purposes
# figure()
# plot(xWS[:,4])
# ylabel("Velocity warm start")
# show()

# println("********** prediction horizon: $(N)")


println("solving finite time optimal control problem ...")
sp10, up10, scaleTime10, exitflag10, time10, lp10, np10 = ParkingSignedDist(s0,sF,N,Ts,L,ego,XYbounds,nOb,vObMPC,AOb,bOb,fixTime,xWS,uWS)

println("****************** exitflag: $(exitflag10) [1=Optimal; 0=Error] ********************")


# println("************ time scale after optimization: $(scaleTime10[1])")
# println("************ Ts: $(Ts)")

println("interpolating solution ....")
tDomOpt 	= unshift!(cumsum(scaleTime10*Ts),0)
pop!(tDomOpt)
tDom 		= 0:time_step:floor(tDomOpt[end-1],1)
fsp10 		= intp.interp1d(tDomOpt, sp10)
fup10 		= intp.interp1d(tDomOpt[1:end-1], up10)
sp10Ref 	= fsp10(tDom)
up10Ref 	= fup10(tDom)
dp10Ref 	=  cumsum( sqrt.( diff(sp10Ref[1,:]).^2 .+ diff(sp10Ref[2,:]).^2 ) )
unshift!(dp10Ref,0)



println("saving solution ...")
save(base_path*"/data/parkPath.jld","sp10",sp10, 
	                           "up10",up10,
	                           "scaleTime10",scaleTime10,
	                           "sp10Ref",sp10Ref,
	                           "up10Ref",up10Ref,
	                           "dp10Ref",dp10Ref,
	                           "time_step",time_step,
	                           "ox", ox,	# added by GXZ for debugging
	                           "oy", oy, 	# added by GXZ for debugging
	                           "xWS", xWS,	# added by GXZ, smoothened Hybrid A*
	                           "uWS", uWS, 	# added by GXZ, smoothened Hybrid A*
	                           )

### comparison with Hybrid A* ###
println("plotting solution ....")
title("Trajectory Comparison")
hold(1)
# plot obstacles
for j = 1 : nOb 
	for k = 1 : vOb[j] -1
		plot([lOb[j][k,:][1],lOb[j][k+1,:][1]] , [lOb[j][k,:][2],lOb[j][k+1,:][2]] ,"k")
	end
end

plot(sp10[1,:],sp10[2,:], "-b",  label="H-OBCA")
plot(rx, ry, "--r",  label="Hybrid A*")
plot(s0[1],s0[2],"ob")
plot(sF[1],sF[2],"ob")
legend()
axis("equal")
show()

figure()
plot(up10[1,:], label="OPT")
plot(uWS[:,1], label="WS")
title("Steering U1")
legend()
show()

figure()
title("Acceleration U2")
plot(up10[2,:], label = "OPT")
plot(uWS[:,2], label = "WS")
legend()
show()

# for debugging purposes
figure()
title("Velocity")
plot(sp10[4,:], label = "OPT")
plot(xWS[:,4], label = "WS")
legend()
show()

totTime = timeHybAstar+time10	# total execution time of H-OBCA
println("Total run time:             \t\t" , totTime, " s")
println("\tHybrid A* time:            \t", timeHybAstar, " s")
println("\toptimization (OBCA) time:  \t", time10, " s")
