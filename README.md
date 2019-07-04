# pose_to_udp

ROS package that sends 6 DoF poses to a UDP connection from a ROS [geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html) topic.


## Usage

For starting the ROS -> UDP bridge, run:
```
roslaunch pose_to_udp pose_to_udp.launch
```

Depending on the parameters set in the launch file above, the node can send poses with information containing data about 3 DoF (X,Y,Yaw) or 6 DoF using euler angles (X,Y,Z,Yaw,Pitch,Roll) or quaternions (X,Y,Z,Qx,Qy,Qz,Qw).

Each value sent over the UDP packet is a 32 bit float in which the translations are in meters and the euler rotations are in radians.

The pose received from the [geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html) topic and can be transformed to another coordinate system using TF (parameter udp_pose_coordinate_system in the [pose_to_udp.launch](launch/pose_to_udp.launch)).


## Testing UDP communications

For sending packets with strings from the command line:
```
netcat -u 192.168.1.1 1337
```

For echoing packages in the command line:
```
socat -v UDP4-RECVFROM:1337,fork exec:'/bin/cat'
```
