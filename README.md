# Autonomous Robots Gaming System

## Launch :
Environment on PC:  
export ROS_MASTER_URI=http://IP_ROBOT:PORT_ROBOT  
export ROS_HOSTNAME=IP_PC  

Environment on robot:  
export ROS_MASTER_URI=http://IP_ROBOT:PORT_ROBOT  
export ROS_HOSTNAME=IP_ROBOT  

On the robot:  
roslaunch turtlebot_bringup minimal.launch // this line starts the roscore  
roslaunch turtlebot_bringup 3dsensor.launch // this line starts the kinect  

On the PC:  
python srcipt/augmented_reality.py

## Notes :
augmented_reality.py script needs scipy, for that :  
pip install scipy (if failed, may needs a pip upgrade)  
