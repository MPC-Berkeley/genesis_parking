#!/usr/bin/env julia

println("loading modules ...")
using RobotOS
using JuMP
using Ipopt
using NPZ
using JLD
using PyCall
using Polynomials
@rosimport genesis_parking.msg: mpcSol
@rosimport genesis_parking.msg: left_path_boundary
@rosimport genesis_parking.msg: right_path_boundary
@rosimport genesis_parking.msg: state_est
@rosimport std_msgs.msg: Float64MultiArray
@rosimport std_msgs.msg: Float32
@rosimport std_msgs.msg: Bool
@rosimport std_msgs.msg: UInt8
rostypegen()
using genesis_parking.msg
using std_msgs.msg
include("mpcPathFollowing.jl")
@pyimport rospkg

obstacle = false

function euclideanDistance(s1::Array{Float64},s2::Array{Float64})
	dx = s1[1] - s2[1]
	dy = s1[2] - s2[2]
	return sqrt( dx^2 + dy^2 )
end

function update_state(msg::state_est, 
					sCurr::Array{Float64}, 
					sPrev::Array{Float64}, 
					dist::Array{Float64}, 
					sRef::Array{Float64,2},
					uRef::Array{Float64,2},
					sp10Ref::Array{Float64,2}, 
					up10Ref::Array{Float64,2},
					dp10Ref::Array{Float64},
					N::Int64)
    # get state
    sPrev[:] = copy(sCurr)
    sCurr[:] = [msg.x , msg.y , msg.psi , msg.v]

    # compute distance travelled in previous time step
    dx 	 		= sp10Ref[1,:] - msg.x
    dy 			= sp10Ref[2,:] - msg.y
    ds 			= sqrt.( dx.^2 + dy.^2)
    idx 		= indmin(ds)

    #delD  	   = euclideanDistance(sCurr, sPrev)
    #dist[:]    = dist[1] + delD
    #idx 	   = indmin( (dp10Ref - dist[1]).^2 )

	sRef[:,:] = copy( sp10Ref[:,idx:idx+N] )
	uRef[:,:] = copy( up10Ref[:,idx:idx+N-1] )
end

function update_obstacle(msg::BoolMsg)
	global obstacle
	obstacle = msg.data
end

function main()
    # initiate node, set up publisher / subscriber topics
    println("loading parameters ...")
    rospack     		= rospkg.RosPack()
    base_path      		= rospack[:get_path]("genesis_parking")
	vehicle_data 		= npzread(base_path*"/data/vehicle_data.npz")
	boundary_data 		= npzread(base_path*"/data/boundary_data.npz")

    s0 			= vehicle_data["initial_state"]
    sF 			= vehicle_data["final_state"]
    Lf 			= vehicle_data["length_front"]  # center of mass to front axle
	Lr 			= vehicle_data["length_rear"]  	# center of mass to rear axle
	L 			= Lf+Lr  						# wheel base (i.e., distance from rear axel to front axel) 

    N 			= get_param("/horizon_preview")
    Q 			= get_param("/state_penalty_matrix")
    R 			= get_param("/input_penalty_matrix")
    stateDim 	= size(s0)[1]
    inputDim 	= 2
    Q 			= reshape(Q,(stateDim, stateDim))
    R 			= reshape(R,(inputDim, inputDim))
    
    # vehicle dimensions
	side_buffer = 0.3
	Lf_buffer = 0.845
	Lr_buffer = 1.135
	Lf_bumber = 2.366
	Lr_bumber = 2.633
	wl = 1.89/2
	wr = 1.89/2
	


	println("loading pre-computed solution from path planner ...")
	data 		= load(base_path*"/data/parkPath.jld")
	sp10 		= data["sp10"]
	up10 		= data["up10"]
	scaleTime10 = data["scaleTime10"]
	sp10Ref 	= data["sp10Ref"]
	up10Ref 	= data["up10Ref"]
	dp10Ref 	= data["dp10Ref"]
	Ts 			= data["time_step"]
	sp10Rest 	= sp10[:,end]*ones(1,N+1)
	up10Rest 	= [0,0]*ones(1,N)
	sp10Ref 	= hcat(sp10Ref, sp10Rest)
	up10Ref 	= hcat(up10Ref, up10Rest)
	sCurr 		= copy(s0)
	sPrev 		= copy(s0)
	sRef 		= copy(sp10Ref[:,1:N+1] )
	uRef 		= copy(up10Ref[:,1:N] )
	dist  		= [0.0]
	solveMPC 	= true

	println("running first optimization with a horizon preview of N = ", N, " steps ...")
	mdlParams   = MdlParams(L,Ts)
	mpcParams   = MpcParams(N,Q,R)
	sol         = MpcSol()	
	mdl    		= MpcModel(mpcParams,mdlParams)



	println("initilizing node ...")
    init_node("controller")
    mpc_pub 			= Publisher("/mpcSolution", mpcSol, queue_size=1)
    acc_pub   			= Publisher("/control/accel", Float32Msg, queue_size=2)
    steer_pub 			= Publisher("/control/steer_angle", Float32Msg, queue_size=2)
    acc_enable_pub   	= Publisher("/control/enable_accel", UInt8Msg, queue_size=2, latch=true)
    steer_enable_pub 	= Publisher("/control/enable_spas",  UInt8Msg, queue_size=2, latch=true)
    
    solveMPC_pub		= Publisher("/solveMPC", BoolMsg, queue_size=1)
    
    # NOT in use
    #left_boundary_pub	= Publisher("/left_path_boundary", Float64MultiArray, queue_size=1)
    #right_boundary_pub	= Publisher("/right_path_boundary", Float64MultiArray, queue_size=1)


	publish(acc_enable_pub, UInt8Msg(1))
	publish(steer_enable_pub, UInt8Msg(1))
	


    s1 = Subscriber{state_est}("/state_estimate", update_state, (sCurr,sPrev,dist,sRef,uRef,sp10Ref,up10Ref,dp10Ref,N), queue_size=1)
    s2 = Subscriber{BoolMsg}("/obstacle", update_obstacle, queue_size=1)
    
    loop_rate = Rate( Int(1.0/Ts) )

	println("running mpc ...")

	uCurr = zeros(2)

    while ! is_shutdown()
    	global obstacle
    	println(obstacle)
    	if euclideanDistance(sCurr, sF) > 0.8 && solveMPC
    		if !obstacle
				println("current state:", sCurr)
				println("current input:", uCurr) 
				SolveMpcProblem(mdl, sol, sCurr, uCurr, sRef, uRef)
				println(sol.solverStatus)
				publish(mpc_pub, mpcSol( sol.zOL[1,:], sol.zOL[2,:], sol.zOL[3,:] , sol.zOL[4,:], sol.uOL[1,:], sol.uOL[2,:]) )
				#err = sol.zOL - sRef
				#println(norm(err))
				
				###################################### NOT IN USE #################################################################
				lengthPath = size(sRef,2)
				leftSide = Array{Float64,2}(1, 2)
				rightSide = Array{Float64,2}(1, 2)
				radius = 2.366
				
				sLocalTemp = [0.0, 0.0, 0.0]

				for i = 1 : lengthPath
					if i != lengthPath
						radius = radius + euclideanDistance(sRef[:,i], sRef[:,i+1])
					end
					
					sLocalTemp[1] = (sRef[1,i] - sCurr[1]) * sin(sCurr[3]) - (sRef[2,i] - sCurr[2]) * cos(sCurr[3])
					sLocalTemp[2] = (sRef[1,i] - sCurr[1]) * cos(sCurr[3]) + (sRef[2,i] - sCurr[2]) * sin(sCurr[3])
					sLocalTemp[3] = pi/2 - sCurr[3] + sRef[3,i]
					
					xRearLeft	= sLocalTemp[1] - Lr_buffer*cos(sLocalTemp[3]) - (wl + side_buffer) * sin(sLocalTemp[3])
					yRearLeft	= sLocalTemp[2] - Lr_buffer*sin(sLocalTemp[3]) + (wl + side_buffer) * cos(sLocalTemp[3])
	
					xRearRight	= xRearLeft + (wl + wr + 2 * side_buffer) * sin(sLocalTemp[3])
					yRearRight	= yRearLeft - (wl + wr + 2 * side_buffer) * cos(sLocalTemp[3])
	
					xFrontLeft	= xRearLeft + (Lr_bumber+Lf_bumber) * cos(sLocalTemp[3])
					yFrontLeft	= yRearLeft + (Lr_bumber+Lf_bumber) * sin(sLocalTemp[3])
	
					xFrontRight	= xFrontLeft + (wl + wr + 2 * side_buffer) * sin(sLocalTemp[3])
					yFrontRight	= yFrontLeft - (wl + wr + 2 * side_buffer) * cos(sLocalTemp[3])
	
					rearLeft	= [xRearLeft,yRearLeft]
					frontLeft	= [xFrontLeft,yFrontLeft]
					rearRight	= [xRearRight,yRearRight]
					frontRight	= [xFrontRight,yFrontRight]
	
					if i == 1
						leftSide[1, :]	= rearLeft'
						leftSide[1, :]	= frontLeft'
						rightSide[1, :]	= rearRight'
						rightSide[1, :]	= frontRight'
					else
						leftSide	= [leftSide;rearLeft']
						leftSide	= [leftSide;frontLeft']
						rightSide	= [rightSide;rearRight']
						rightSide	= [rightSide;frontRight']
					end
				end
				
				if euclideanDistance(sCurr, sF) > 2
					leftSideCoeff = coeffs(polyfit(leftSide[:, 1], leftSide[:, 2], 3))
					rightSideCoeff = coeffs(polyfit(rightSide[:, 1], rightSide[:, 2], 3))
				else
					leftSideCoeff = coeffs(polyfit(leftSide[:, 1], leftSide[:, 2], 2))
					leftSideCoeff = [0; leftSideCoeff]
					rightSideCoeff = coeffs(polyfit(rightSide[:, 1], rightSide[:, 2], 2))
					rightSideCoeff = [0; rightSideCoeff]
				end

				#################################################### END #########################################################

				publish(acc_pub, Float32Msg(sol.acc) )
				publish(steer_pub, Float32Msg(sol.df) )
				publish(solveMPC_pub, BoolMsg(solveMPC))

				# NOT IN USE
				#left_msg = Float64MultiArray()
				#right_msg = Float64MultiArray()
				#left_msg.data = [leftSideCoeff[1], leftSideCoeff[2], leftSideCoeff[3], leftSideCoeff[4], radius]
				#right_msg.data = [rightSideCoeff[1], rightSideCoeff[2], rightSideCoeff[3], rightSideCoeff[4], radius]
				#publish(left_boundary_pub, left_msg)
				#publish(right_boundary_pub, right_msg)



				uCurr = [sol.df ; sol.acc]
			else
				uCurr = [sol.df ; -5.0]
				publish(acc_pub, Float32Msg(-5.0))
				publish(steer_pub, Float32Msg(sol.df))
			end
    	else
    		solveMPC = false
    		SolveMpcProblem(mdl, sol, sCurr, uCurr, sRef, uRef)
    		println(sol.solverStatus)
    		publish(mpc_pub, mpcSol( sol.zOL[1,:], sol.zOL[2,:], sol.zOL[3,:] , sol.zOL[4,:], sol.uOL[1,:], sol.uOL[2,:]) )
    		

    		###################################### NOT IN USE #################################################################
    		lengthPath = size(sRef,2)
			leftSide = Array{Float64,2}(1, 2)
			rightSide = Array{Float64,2}(1, 2)
			radius = 2.366
			
			sLocalTemp = [0.0, 0.0, 0.0]

			for i = 1 : lengthPath
				if i != lengthPath
					radius = radius + euclideanDistance(sRef[:,i], sRef[:,i+1])
				end
					sLocalTemp[1] = (sRef[1,i] - sCurr[1]) * sin(sCurr[3]) - (sRef[2,i] - sCurr[2]) * cos(sCurr[3])
					sLocalTemp[2] = (sRef[1,i] - sCurr[1]) * cos(sCurr[3]) + (sRef[2,i] - sCurr[2]) * sin(sCurr[3])
					sLocalTemp[3] = pi/2 - sCurr[3] + sRef[3,i]
					
					xRearLeft	= sLocalTemp[1] - Lr_buffer*cos(sLocalTemp[3]) - (wl + side_buffer) * sin(sLocalTemp[3])
					yRearLeft	= sLocalTemp[2] - Lr_buffer*sin(sLocalTemp[3]) + (wl + side_buffer) * cos(sLocalTemp[3])
	
					xRearRight	= xRearLeft + (wl + wr + 2 * side_buffer) * sin(sLocalTemp[3])
					yRearRight	= yRearLeft - (wl + wr + 2 * side_buffer) * cos(sLocalTemp[3])
	
					xFrontLeft	= xRearLeft + (Lr_bumber+Lf_bumber) * cos(sLocalTemp[3])
					yFrontLeft	= yRearLeft + (Lr_bumber+Lf_bumber) * sin(sLocalTemp[3])
	
					xFrontRight	= xFrontLeft + (wl + wr + 2 * side_buffer) * sin(sLocalTemp[3])
					yFrontRight	= yFrontLeft - (wl + wr + 2 * side_buffer) * cos(sLocalTemp[3])
	
					rearLeft	= [xRearLeft,yRearLeft]
					frontLeft	= [xFrontLeft,yFrontLeft]
					rearRight	= [xRearRight,yRearRight]
					frontRight	= [xFrontRight,yFrontRight]
	
				if i == 1
					leftSide[1, :]	= rearLeft'
					leftSide[1, :]	= frontLeft'
					rightSide[1, :]	= rearRight'
					rightSide[1, :]	= frontRight'
				else
					leftSide	= [leftSide;rearLeft']
					leftSide	= [leftSide;frontLeft']
					rightSide	= [rightSide;rearRight']
					rightSide	= [rightSide;frontRight']
				end
			end

			leftSideCoeff = coeffs(polyfit(leftSide[:, 1], leftSide[:, 2], 1))
			rightSideCoeff = coeffs(polyfit(rightSide[:, 1], rightSide[:, 2], 1))

			#################################################### END #########################################################
    		
    		publish(acc_pub, Float32Msg(-5.0) )
    		publish(steer_pub, Float32Msg(sol.df) )
    		publish(solveMPC_pub, BoolMsg(solveMPC))

    		# NOT IN USE
    		#left_msg = Float64MultiArray()
    		#right_msg = Float64MultiArray()
    		#left_msg.data = [leftSideCoeff[1], leftSideCoeff[2], 0, 0, radius]
    		#right_msg.data = [rightSideCoeff[1], rightSideCoeff[2], 0, 0, radius]
    		#publish(left_boundary_pub, left_msg)
    		#publish(right_boundary_pub, right_msg)

    		
    		uCurr = [sol.df ; -5.0]
    		println("current state:", sCurr)
			println("current input:", uCurr)
    	end

        rossleep(loop_rate)
    end
end

if ! isinteractive()
    main()
end
