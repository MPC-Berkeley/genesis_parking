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
@rosimport std_msgs.msg: UInt32
rostypegen()
using genesis_parking.msg
using std_msgs.msg
include("mpcPathFollowing.jl")
@pyimport rospkg


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

function update_obstacle(msg::BoolMsg, obstacle::Bool)
	obstacle = msg.data
end

function main()
    # initiate node, set up publisher / subscriber topics
    println("loading parameters ...")
    rospack     		= rospkg.RosPack()
    base_path      		= rospack[:get_path]("genesis_parking")
    
    double_maneuver		= false
    
    if double_maneuver == false
		vehicle_data 		= npzread(base_path*"/data/vehicle_data.npz")
		boundary_data 		= npzread(base_path*"/data/boundary_data.npz")
	else
		vehicle_data 		= npzread(base_path*"/data/vehicle_data_2.npz")
		boundary_data 		= npzread(base_path*"/data/boundary_data_2.npz")
	end

    s0 			= vehicle_data["initial_state"]
    sF 			= vehicle_data["final_state"]
    Lf 			= vehicle_data["length_front"]  # distance from center of rear axel to front edge of car
	Lr 			= vehicle_data["length_rear"]  	# distance from center of rear axel to back edge of car
	L 			= Lf+Lr  						# wheel base (i.e., distance from rear axel to front axel) 
	
	println(sF)
	
	
    N 			= get_param("/horizon_preview")
    Q 			= get_param("/state_penalty_matrix")
    R 			= get_param("/input_penalty_matrix")
    stateDim 	= size(s0)[1]
    inputDim 	= 2
    Q 			= reshape(Q,(stateDim, stateDim))
    R 			= reshape(R,(inputDim, inputDim))
    
    
	side_buffer = 0.3
	Lf_buffer = 0.845
	Lr_buffer = 1.135
	Lf_bumber = 2.366
	Lr_bumber = 2.633
	wl = 1.89/2
	wr = 1.89/2
	
	obstacle = false

	println("loading pre-computed solution from path planner ...")
	
	if double_maneuver == false
		data 		= load(base_path*"/data/parkPath.jld")
	else
		data 		= load(base_path*"/data/parkPath_2.jld")
	end
	
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
    
    state_ref_pub		= Publisher("/state_ref", UInt32Msg, queue_size=1)


	publish(acc_enable_pub, UInt8Msg(1))
	publish(steer_enable_pub, UInt8Msg(1))
	


    s1 = Subscriber{state_est}("/state_estimate", update_state, (sCurr,sPrev,dist,sRef,uRef,sp10Ref,up10Ref,dp10Ref,N), queue_size=1)
    s2 = Subscriber{BoolMsg}("/Obstacle", update_obstacle, (obstacle,), queue_size=1)
    
    loop_rate = Rate( Int(1.0/Ts) )

	println("running mpc ...")

	uCurr = zeros(2)

    while ! is_shutdown()

    	if euclideanDistance(sCurr, sF) > -0.1 && solveMPC
    		if !obstacle
				println("current state:", sCurr)
				println("current input:", uCurr) 
				SolveMpcProblem(mdl, sol, sCurr, uCurr, sRef, uRef)
				println(sol.solverStatus)
				publish(mpc_pub, mpcSol( sol.zOL[1,:], sol.zOL[2,:], sol.zOL[3,:] , sol.zOL[4,:], sol.uOL[1,:], sol.uOL[2,:]) )
				#err = sol.zOL - sRef
				#println(norm(err))
				
				
				dx 	 		= sp10Ref[1,:] - sCurr[1]
    			dy 			= sp10Ref[2,:] - sCurr[2]
    			ds 			= sqrt.( dx.^2 + dy.^2)
    			idx 		= indmin(ds)-1
    			publish(state_ref_pub, UInt32Msg(idx))
				
				
				sLocalTemp = [0.0, 0.0, 0.0]
				publish(acc_pub, Float32Msg(sol.acc) )
				publish(steer_pub, Float32Msg(sol.df) )
				publish(solveMPC_pub, BoolMsg(solveMPC))
				
				uCurr = [sol.df ; sol.acc]
			else
				uCurr = [sol.df ; -3.0]
				publish(acc_pub, Float32Msg(-3.0))
				publish(steer_pub, Fload32Msg(sol.df))
			end
    	else
    		solveMPC = false
    		SolveMpcProblem(mdl, sol, sCurr, uCurr, sRef, uRef)
    		println(sol.solverStatus)
    		publish(mpc_pub, mpcSol( sol.zOL[1,:], sol.zOL[2,:], sol.zOL[3,:] , sol.zOL[4,:], sol.uOL[1,:], sol.uOL[2,:]) )
    		
			sLocalTemp = [0.0, 0.0, 0.0]
			
			dx 	 		= sp10Ref[1,:] - sCurr[1]
    		dy 			= sp10Ref[2,:] - sCurr[2]
    		ds 			= sqrt.( dx.^2 + dy.^2)
    		idx 		= indmin(ds)-1
    		publish(state_ref_pub, UInt32Msg(idx))

    		publish(acc_pub, Float32Msg(-3.0) )
    		publish(steer_pub, Float32Msg(sol.df) )
    		publish(solveMPC_pub, BoolMsg(solveMPC))
    		
    		uCurr = [sol.df ; -3.0]
    	end

        rossleep(loop_rate)
    end
end

if ! isinteractive()
    main()
end
