# rosbag_service

ROSBAG log functionality

## Use
Rosbags are stored in $ME00_FOLDERS/log/rosbags. Compressed saved rosbags are in $ME00_FOLDERS/log/compressed

ros2 launch me00_bringup start_launch.py 

**Activate**

```bash
ros2 service call /me00_1/RosBag/rosbag_start_srv ageve_interfaces/srv/Rosbag "{name: my_bag}
```

**Deactivate**

```bash
ros2 service call /me00_1/RosBag/rosbag_stop_srv std_srvs/srv/Trigger 
```

**Save a rosbag**

```bash
ros2 service call /me00_1/RosBag/rosbag_save_srv ageve_interfaces/srv/Rosbag "{name: my_bag}"
```

## License
All rights reserved to movvo robotics