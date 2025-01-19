#!/usr/bin/bash

tmux new-session -d -s main_session
tmux split-window -v -t main_session:0
tmux split-window -v -t main_session:0
tmux split-window -v -t main_session:0

tmux send-keys -t main_session:0.0 "ros2 launch ras_app_main main.launch.py" C-m
tmux send-keys -t main_session:0.1 "ros2 run ras_bt_framework executor" C-m
tmux send-keys -t main_session:0.2 "ros2 launch ras_moveit moveit_server.launch.py" C-m
tmux send-keys -t main_session:0.3 "ros2 run ras_bt_framework TrajectoryRecordsService.py" C-m

tmux new-window -t main_session:1 -n 'website'
tmux split-window -v -t main_session:1
tmux split-window -v -t main_session:1
tmux split-window -v -t main_session:1

tmux send-keys -t main_session:1.0 "ros2 launch rosbridge_server rosbridge_websocket_launch.xml" C-m

tmux send-keys -t main_session:1.1 "cd /ras_server_app/ras_webapp && npm run dev" C-m

tmux send-keys -t main_session:1.2 "ros2 run ras_transport log_receiver.py" C-m
tmux send-keys -t main_session:1.3 "ros2 run ras_transport mqtt_broker.py" C-m

tmux new-window -t main_session:2 -n 'experiment'
tmux split-window -v -t main_session:2
tmux split-window -v -t main_session:2
tmux split-window -v -t main_session:2

tmux send-keys -t main_session:2.0 "ros2 run ras_bt_framework experiment_service.py" C-m
tmux send-keys -t main_session:2.2 "ros2 run ras_transport iot_sender.py" C-m
tmux send-keys -t main_session:2.3 "ros2 run ras_bt_framework FakeGripperServer.py" C-m


tmux new-window -t main_session:3 -n 'debugging'
tmux split-window -v -t main_session:3
tmux split-window -v -t main_session:3

tmux send-keys -t main_session:3.0 "ros2 run ras_sim spawn_model_node" C-m
tmux send-keys -t main_session:3.1 "ros2 run ras_sim spawn_manager.py" C-m


tmux attach-session -t main_session
