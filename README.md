# rosbag_service

ROSBAG log service functionality

## Install
Rosbag service depends on rosbag2 package

git submodule init
git submodule update

**Activate**

```bash
ros2 service call /rosbag2_service/rosbag2_service_node/rosbag_start_srv rosbag2_service_msg/srv/RosbagStart "{path: '/path/to/rosbag/folder/to/create'}
```

**Deactivate**

```bash
ros2 service call /rosbag2_service/rosbag2_service_node/rosbag_stop_srv std_srvs/srv/Trigger 
```

**Save a rosbag**

```bash
ros2 service call /rosbag2_service/rosbag2_service_node/rosbag_save_srv rosbag2_service_msg/srv/RosbagSave "{path: '/path/to/rosbag/folder/to/compress', compressed_path: '/path/to/compressed/folder/to/create'}"
```

## License
2023 Movvo Robotics - MIT License (see license file) 