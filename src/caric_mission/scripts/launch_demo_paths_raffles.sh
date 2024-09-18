# Build the workspace
catkin build;

# Launch the drones

(
    sleep 5;
    wait;
    
    (
        rosservice call /raffles/traj_gennav/readfile;
        rosservice call /raffles/traj_gennav/execute_path;
    )
) & \
roslaunch caric_mission demo_paths_raffles.launch