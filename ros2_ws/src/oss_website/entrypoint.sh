#!/bin/bash

IOT_CONFIG_FILE="/workspaces/robo_arm_ws/src/xarm_cpp_custom/iot_certs_and_config/iot_config.json"

# Start a new tmux session named 'main_session'
tmux new-session -d -s main_session
tmux split-window -v -t main_session0
tmux split-window -v -t main_session:0
tmux send-keys -t main_session:0.0 "cd /workspaces/robo_arm_ws/src/xarm_cpp_custom/python && python3 aws_status_receiver.py" C-m
tmux send-keys -t main_session:0.1 "cd /workspaces/robo_arm_ws/src/xarm_cpp_custom/python && python3 aws_hard_reset.py" C-m

# Create a new window for ROS2-related tasks
tmux new-window -t main_session:1 -n 'ros2_window'
tmux split-window -v -t main_session:1
tmux split-window -v -t main_session:1

# Run ROS2 launch file in the first window
tmux send-keys -t main_session:1.0 "ros2 launch rosbridge_server rosbridge_websocket_launch.xml" C-m

# Run npm run dev in the second window
tmux send-keys -t main_session:1.1 "npm run dev" C-m

# Run node read_yaml.js in the third window
tmux send-keys -t main_session:1.2 "cd /workspaces/robo_arm_ws/src/xarm_cpp_custom/python && python3 log_receiver.py" C-m

# Create a new window for the xarm and spawn model
tmux new-window -t main_session:2 -n 'xarm_window'
tmux split-window -v -t main_session:2

# Run xarm_gz in the first pane
tmux send-keys -t main_session:2.0 "xarm_gz" C-m

# Run ros2 run spawn_gz spawn_model_node in the second pane
tmux send-keys -t main_session:2.1 "ros2 run spawn_gz spawn_model_node" C-m

# Create a new window for the experiment execution
tmux new-window -t main_session:3 -n 'experiment_window'
tmux split-window -v -t main_session:3
tmux split-window -v -t main_session:3

# Run execute_experiment.py in the first pane
tmux send-keys -t main_session:3.0 "ros2 run oss_core execute_experiment.py" C-m

# Run ExecuteSim in the second pane
tmux send-keys -t main_session:3.1 "ros2 run xarm_cpp_custom ExecuteSim" C-m

# Run iot_sender.py in the third pane
tmux send-keys -t main_session:3.2 "cd /workspaces/robo_arm_ws/src/xarm_cpp_custom/python && python3 iot_sender.py --ros-args --param path_for_config:=$IOT_CONFIG_FILE" C-m

# Attach to the tmux session to view the windows
tmux attach-session -t main_session
