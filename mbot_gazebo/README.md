#Commands

##To spawn the urdf in gazebo
`rosrun gazebo_ros spawn_model -file `rospack find mbot_description`/urdf/mbot.urdf -urdf -x 0 -y 0 -z 1 -model mbot`

##To start rviz and view different sensors
`rosrun rviz rviz` or `rviz`

##Test robot movement using rqt steering, publish data on topics, plotting etc.
`rqt`

##Launch package
`roslaunch mbot_gazebo mbot.launch --screen`

##Additional packages required      
- ros-kinetic-gmapping     
- ros-kinetic-global-planner    
- ros-kinetic-effort-controllers    
- ros-kinetic-gazebo-ros      
- ros-kinetic-gazebo-msgs    
- ros-kinetic-gazebo-plugins    
- ros-kinetic-gazebo-ros-control    
- ros-kinetic-amcl  
- ros-kinetic-move-base      