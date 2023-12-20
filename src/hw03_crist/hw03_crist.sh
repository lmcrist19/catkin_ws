rosservice call /clear
rosservice call /turtle1/teleport_absolute 1 1 0
rosservice call /clear


rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "linear:
 x: 0.0
 y: 4.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 0.0"
 
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "linear:
 x: 1.0
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 0.0"
 
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "linear:
 x: 3.14
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: -3.14"

rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "linear:
 x: 0.0
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 2.1"
 
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "linear:
 x: 2.26
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 0.0"
 
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "linear:
 x: 0.0
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 1.05"
 
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "linear:
 x: 2.0
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 0.0"

rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "linear:
 x: 1.57
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 1.57"

rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "linear:
 x: 2.0
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 0.0"
 
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "linear:
 x: 3.14
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 3.14"
 
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "linear:
 x: 2.0
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 0.0"
 
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "linear:
 x: 1.57
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 1.57"
 
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "linear:
 x: 3.0
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 0.0"
 
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "linear:
 x: 3.14
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 3.14"
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "linear:
 x: 3.14
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: -3.14"

rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "linear:
 x: 2.0
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 0.0"
