catkin_make --source Modules/common/msgs --build build/msgs
catkin_make --source Modules/control --build build/control
catkin_make --source Modules/mission --build build/mission

catkin_make --source Modules/test_nlink --build build/test_nlink
catkin_make --source Experiment --build build/prometheus_experiment
