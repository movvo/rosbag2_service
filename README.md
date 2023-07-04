# rosbag_service

ROSBAG log service functionality

## Install
git submodule init
git submodule update

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