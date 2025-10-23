# auto_drive_car
10.22 自动小车作业


巡线
`ros2 launch auto_drive auto_drive_env.launch.py yaw:=1.5708`
`ros2 run auto_drive auto_drive_node`

信号灯
`ros2 launch auto_drive auto_drive_env.launch.py yaw:=1.5708`
`ros2 run auto_drive auto_drive_node_traffic `

slam
依赖
`sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-turtlebot3* ros-jazzy-nav2-*`
开始建图
`ros2 launch auto_drive auto_drive_env.launch.py world:=drive_track slam:=true rviz:=true`
保存地图
`ros2 run nav2_map_server map_saver_cli -f /data/auto_driver_car/install/auto_drive/share/auto_drive/maps/my_map`
运行地图
`ros2 launch auto_drive auto_drive_env.launch.py   localization:=true   map:=/data/auto_driver_car/install/auto_drive/share/auto_drive/maps/my_map.yaml   rviz:=true   nav2:=true`