source ws/install/setup.bash || source /opt/ros/humble/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

export CYCLONEDDS_URI="file://$(pwd)/cyclonedds.xml"

alias rosenv='printenv | grep -E "ROS|RMW_IMPLEMENTATION|AMENT|CYCLONEDDS_URI"'
