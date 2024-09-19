# Build the workspace
catkin build;

# Launch the drones

(
    sleep 5;
    wait;
    
    (
        rosservice call /jurong/traj_gennav/readfile;
        rosservice call /jurong/traj_gennav/execute_path;
    )
) & \
roslaunch caric_mission demo_paths_jurong.launch