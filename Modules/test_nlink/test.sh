#launch px4 node
gnome-terminal --title="$file" --window -x bash -c "roslaunch mavros px4.launch fcu_url:="/dev/ttyUSB0:921600";" 
#launch uwb node
gnome-terminal --title="$file" --window -x bash -c "roslaunch nlink_parser linktrack.launch;" 
#run the listen node
gnome-terminal --title="$file" --window -x bash -c "rosrun test_nlink linktrack_example;" 

