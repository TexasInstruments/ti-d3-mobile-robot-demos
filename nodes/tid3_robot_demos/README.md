TI-D3 Mobile Robot Demos
========================
This contains ROS launch files for the mobile robot demos.


## Launch Example
RDA4:
```
roslaunch tid3_robot_demos gscam_objdet_cnn.launch framerate:=15 width:=1920 height:=1080 video_id:=18 subdev_id:=7
roslaunch tid3_robot_demos gscam_objdet_cnn.launch framerate:=15 width:=1280 height:=720 video_id:=18 subdev_id:=7
roslaunch tid3_robot_demos gscam_objdet_cnn.launch framerate:=15 width:=960 height:=540 video_id:=18 subdev_id:=7
```

Visualization on PC:
```
roslaunch ti_viz_nodes rviz_objdet_cnn.launch width:=1920 height:=1080
roslaunch ti_viz_nodes rviz_objdet_cnn.launch width:=1280 height:=720
roslaunch ti_viz_nodes rviz_objdet_cnn.launch width:=960 height:=540
```