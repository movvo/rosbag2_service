# rosbag_service

ROSBAG log service functionality. It has 3 services
* Start rosbag: Starts rosbag and logs all the topics indicated in the config file (params.yaml)
* Stop rosbag: Stops the current rosbag
* Save rosbag: Saves the given rosbag in another folder compressed using zstd.

## Install
Rosbag service depends on rosbag2 package

git submodule init
git submodule update

**Start**

```bash
ros2 service call /rosbag2_service/rosbag2_service_node/rosbag_start_srv rosbag2_service_msg/srv/RosbagStart "{path: '/path/to/rosbag/folder/to/create'}
```

**Stop**

```bash
ros2 service call /rosbag2_service/rosbag2_service_node/rosbag_stop_srv std_srvs/srv/Trigger 
```

**Save a compressed rosbag**

```bash
ros2 service call /rosbag2_service/rosbag2_service_node/rosbag_save_srv rosbag2_service_msg/srv/RosbagSave "{path: '/path/to/rosbag/folder/to/compress', compressed_path: '/path/to/compressed/folder/to/create'}"
```

## License
2023 Movvo Robotics - MIT License (see license file) 
