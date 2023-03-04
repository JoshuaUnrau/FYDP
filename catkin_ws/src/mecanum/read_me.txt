source devel/setup.bash

catkin_make

ros command to teleop robot:

rosrun teleop_twist_keyboard teleop_twist_keyboard.py

ros launch command to launch all nodes

roslaunch mecanum robot.launch

how to send messages to individual nodes using rostopic pub

rostopic pub my_topic std_msgs/String "hello there"

eg publish angle of 20 degrees to servo

rostopic pub servo std_msgs/Int32 20

eg ask lifter to go up

#Middle 0, Up 1, Down 2
rostopic pub limit_switch std_msgs/Int32 0


#Stop 0, Up 1, Down 2, Moving 3
rostopic pub desired_lifter_state std_msgs/Int32 1

rosrun mecanum ardunioInterface.py

#cd to launch dir first
cd src/mecanum/launch/
roslaunch everythingExceptPositionController.launch
rosrun mecanum position_controller.py

rosrun pwm pwm.py

#Middle = 0	Up = 1 Down = 2
rostopic echo limit_switch

#Motors test
rosrun mecanum mecanum.py