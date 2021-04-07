catkin_make --source Modules/common/msgs --build build/msgs
catkin_make --source Modules/control --build build/control
catkin_make --source Modules/mission --build build/mission

catkin_make --source Simulator/gazebo_simulator --build build/prometheus_gazebo
