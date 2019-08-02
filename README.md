# Genesis Parking

## Introduction:

This project aims to achieve autonomous parking in three different parking scenarios, including both forward and reverse parking in perpendicular parking spot and parallel parking, with the help of optimization-based collision avoidance planning algorithm, MPC and LQR controllers. First, the planning algorithm provides a path to a desired spot within a certain safety region based on differential GPS data. Then, the MPC or LQR controller tracks the path to ensure that the car parks into the spot. For the forward perpendicular parking, MPC controller was used for its tracking quality and foresight of the path. Later on for the reverse parking and parallel parking, the controller was switched to LQR due to physical limitations of the car and calculation speed. 

## Workflow:

1. get_position.launch: Call the launch file with the input “initial” or “final” and it will record the current state of the car including longitude, latitude and yaw angle as initial or final state. 

2. get_boundary.launch: It adjusts for the discrepancy between google map coordinates and on-board gps coordinates and saves the recorded or hardcoded coordinates as initial position, selected parking spot as final position and the boundaries of the parking lot as obstacles. It also visualizes the everything. 

3. get_path.launch: This calls the optimization-based collision avoidance (OBCA) planner to plan an optimal path using the positions and obstacles saved in the previous step. This can take a long time depending on the parameters. 

4. simulator.launch: Before running on the real car, it is recommended to run the simulator to make sure all everything is working and parking quality of generally acceptable. Note that some of the tuning parameters will need to be changed on the real car. 

5. visualizer.launch: When repeating the parking experiment, it is recommended to record the coordinates and plan the path only once, since it might take a long time to plan. The visualizer shows the planned path and the car’s position in real time, so that the car can return to roughly the starting position recorded previously. 

6. parking.launch: Calls the corresponding controller to track the path while visualizing the car’s position.

## Tuning: 

path_planner.yaml: parking lot and parking spot info, car geometry, time step and google map coordinates and gps measurement used for offset calculation

parking.yaml: state and input cost for MPC controller

mpcPathFollowing.jl: delta u cost for MPC controller

lqr.py: Q and R matrix, parking tolerance

get_path.jl, ParkingSignedDist.jl & hybrid_a_star.jl: OBCA planner

simulator.py: all of the visualization & real time display

