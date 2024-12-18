#!/usr/bin/bash

tmux new-session -d -s perception_session
tmux split-window -v -t perception_session:0
tmux split-window -v -t perception_session:0

tmux send-keys -t perception_session:0.0 "python3 /ras_sim_lab/ros2_ws/src/ras_sim/ras_sim/camera.py" C-m
tmux send-keys -t perception_session:0.1 "python3 /ras_sim_lab/ros2_ws/src/ras_sim/ras_sim/logging_server.py" C-m

tmux attach-session -t perception_session