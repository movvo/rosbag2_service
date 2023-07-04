/*
   Copyright 2023 @ MOVVO ROBOTICS
   ---------------------------------------------------------
   Authors: Bernat Gaston, Maria Mercadé, Martí Bolet
   Contact: support.idi@movvo.eu
*/

#include "rosbag2_service.hpp"

int main(int argc, char * argv[]) {
    //init rclcpp
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    //Init dependencies
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("rosbag2_service_node");
    auto rosbag = std::make_shared<rosbag2_service::Rosbag2Service>(nh);
    //Spin
    executor.add_node(nh);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}