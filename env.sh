source /opt/ros/humble/setup.bash
export OSS_APP_PATH=$(realpath $(dirname "${BASH_SOURCE[0]}"))
export OSS_WORKSPACE_PATH=$OSS_APP_PATH/ros2_ws
export IGN_GAZEBO_RESOURCE_PATH=$OSS_APP_PATH/assets/objects/old_models:$OSS_APP_PATH/third_party/gazebo_models:${IGN_GAZEBO_RESOURCE_PATH}
source $OSS_WORKSPACE_PATH/install/setup.bash
source ~/.nvm/nvm.sh
export ROS_DOMAIN_ID=2