# pose_to_udp

ROS package that sends 6 DoF poses to a UDP connection from a ROS sensor_msgs/PoseStamped


## Testing UDP

For sending packets with strings from the command line:
```
netcat -u 192.168.1.1 1337
```

For echoing packages in the command line:
```
socat -v UDP4-RECVFROM:1337,fork exec:'/bin/cat'
```
