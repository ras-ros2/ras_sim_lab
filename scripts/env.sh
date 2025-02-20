source /opt/ros/humble/setup.bash
export RAS_APP_NAME=server
export RAS_APP_NAME_FULL=ras_${RAS_APP_NAME}_app
export RAS_APP_PATH=$(realpath $(dirname "${BASH_SOURCE[0]}")/..)
export RAS_WORKSPACE_PATH=$RAS_APP_PATH/ros2_ws
export IGN_GAZEBO_RESOURCE_PATH=$RAS_APP_PATH/assets/objects/old_models:$RAS_APP_PATH/third_party/gazebo_models:${IGN_GAZEBO_RESOURCE_PATH}
source $RAS_WORKSPACE_PATH/install/setup.bash
source ~/.nvm/nvm.sh
export IGN_PARTITION=$RAS_APP_NAME_FULL
export ROS_DOMAIN_ID=2

# include the ras_cli_setup.sh
source $RAS_APP_PATH/scripts/ras_cli_setup.sh

ras_app() { if [ -e /tmp/.RAS_RUN ]
    then
        echo App is Already Running
    else
        echo Starting App
        touch /tmp/.RAS_RUN
        $RAS_APP_PATH/scripts/run.sh
        rm /tmp/.RAS_RUN
    fi
}

ras_kill() { tmux kill-session -t main_session ; }
