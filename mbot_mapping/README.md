## Before starting any node set sim_time to true   
`rosparam set use_sim_time true`   

## Start gmapping   
`rosrun gmapping slam_gmapping scan:=/hbot/laser/scan _odom_frame:=odom _linearUpdate:=0.1 _angularUpdate:=0.1 _minimumScore:=50.0 _maxRange:=5`

## To save the map   
`rosrun map_server map_saver`   

## Creating bag files  
`rosbag record -O mylaserdata /base_scan /tf`  

## Playing rosbag files  
`rosbag play --clock <name of the bag>`   

##Watching map generation  
`rviz` with map as fixed frame and add map viewer.  



