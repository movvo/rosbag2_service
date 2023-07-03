/*
   Copyright 2023 @ MOVVO ROBOTICS
   ---------------------------------------------------------
   Authors: Bernat Gaston
   Contact: support.idi@ageve.net
*/

#ifndef ROSBAG2_SERVICE_HPP_
#define ROSBAG2_SERVICE_HPP_

// C++ Standard
#include <stdio.h>
#include <iostream>
#include <vector>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_compression_zstd/zstd_compressor.hpp"

//Services
#include "rosbag2_service_msg/srv/rosbag.hpp"

// OWN node
#include "errors.hpp"


namespace rosbag2_service{
/*!
    @class Rosbag
    @brief Rosbag main class to control start,
    stop and save
*/
//================================================
class Rosbag2Service
//================================================
{
 public:
    /*!
      @brief Constructor of the Rosbag class 
      @param[in] sm Injected state machine node dependency
      @param[in] nh Injected rclcpp node dependency
  */
  Rosbag2Service(rclcpp::Node::SharedPtr nh);

  typedef struct
  {
    rclcpp::Parameter autorun;
    rclcpp::Parameter topics;
    rclcpp::Parameter rosbag_folder;
  } configuration;
  configuration parameters; /*< Parameters of the node loaded from yaml*/

 private:
  /*!
      @brief Initialize the node
  */
  bool Initialize();
  /*!
      @brief Start the rosbag service storing information
      @param[in] req request to start the service
      @param[in] response response of the service
  */
  void RosbagStartSrvCallback(
    const std::shared_ptr<rosbag2_service_msg::srv::Rosbag::Request> req,
    std::shared_ptr<rosbag2_service_msg::srv::Rosbag::Response> response);
  /*!
      @brief Stop the rosbag service storing information
      @param[in] response response of the service
  */
  void RosbagStopSrvCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  /*!
      @brief Start the compressing and saving service of rosbags
      @param[in] req request to start the service
      @param[in] response response of the service
  */
  void RosbagSaveSrvCallback(
    const std::shared_ptr<rosbag2_service_msg::srv::Rosbag::Request> req,
    std::shared_ptr<rosbag2_service_msg::srv::Rosbag::Response> response);
  /*!
      @brief Callback, one per topic
      @param[in] msg message to store
      @param[in] topic_name name of the topic
      @param[in] topic_type type of the topic message
  */
  void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg, 
    std::string topic_name, std::string topic_type);
  /*!
      @brief Dynamically reconfigure parameters
      @param[in] parameters parameters to change
  */
  rcl_interfaces::msg::SetParametersResult dyn_reconf_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  rclcpp::Service<rosbag2_service_msg::srv::Rosbag>::SharedPtr rosbagstart_srv_; 
  /*< Variable to store the service of the start rosbag*/
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr rosbagstop_srv_; /*< 
  Variable to store the service of the stop rosbag*/
  rclcpp::Service<rosbag2_service_msg::srv::Rosbag>::SharedPtr rosbagsave_srv_; /*
  < Variable to store the service of the save rosbag*/
  std::vector<rclcpp::GenericSubscription::SharedPtr> subs_vector_; /*< 
  Variable to store the diferent callbacks created for each topic*/
  std::unique_ptr<rosbag2_cpp::Writer> writer_; /*< Writer of the rosbag*/
  rclcpp::Node::SharedPtr nh_;/*< rclcpp node*/
  bool setted_up_ = false; /*< Variable to know if everything is setted up*/
  bool stop_writers_ = false; /*< Variable to stop writters*/
};
} //namespace rosbag2_service
#endif // ROSBAG2_SERVICE_HPP_