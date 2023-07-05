/*
   Copyright 2023 @ MOVVO ROBOTICS
   ---------------------------------------------------------
   Authors: Bernat Gaston, Maria Mercadé, Martí Bolet
   Contact: support.idi@movvo.eu
*/

#include "rosbag2_service.hpp"
#include <vector>
#include "rclcpp/create_generic_subscription.hpp"
#include <filesystem>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/*****************************************************************************
                   ROSBAG NODE
*****************************************************************************/

namespace rosbag2_service{

Rosbag2Service::Rosbag2Service(rclcpp::Node::SharedPtr nh)
{
    //Set up dependencies
    nh_ = nh;

    // Create services
    rosbagstart_srv_ = nh_->create_service<rosbag2_service_msg::srv::RosbagStart>(
        nh_->get_name() + std::string("/rosbag_start_srv"), 
        std::bind(&Rosbag2Service::RosbagStartSrvCallback, this, _1, _2));
    rosbagstop_srv_ = nh_->create_service<std_srvs::srv::Trigger>(
        nh_->get_name() + std::string("/rosbag_stop_srv"), 
        std::bind(&Rosbag2Service::RosbagStopSrvCallback, this, _1, _2));
    rosbagsave_srv_ = nh_->create_service<rosbag2_service_msg::srv::RosbagSave>(
        nh_->get_name() + std::string("/rosbag_save_srv"), 
        std::bind(&Rosbag2Service::RosbagSaveSrvCallback, this, _1, _2));

    // Declare parameters
    nh_->declare_parameter("autorun", true);
    nh_->declare_parameter("topics", std::vector<std::string>());

    // Initialize variables
    Initialize();
}

// /***************************************************************************
//                 OWN FUNCTIONS
// ***************************************************************************/

//=============================================================================
bool Rosbag2Service::Initialize()
//=============================================================================
{
    RCLCPP_INFO(nh_->get_logger(), "Checking parameters...");

    try
    {
        this->parameters.autorun = nh_->get_parameter("autorun");
        this->parameters.topics = nh_->get_parameter("topics");
        setted_up_ = true;
    }
    catch (std::runtime_error &exc)
    {
        RCLCPP_ERROR(nh_->get_logger(), exc.what());
        return false;
    }
    RCLCPP_INFO(nh_->get_logger(), "inicialization done");
    return true;
}

// /***************************************************************************
//                 SERVICE CALLBACK FUNCTIONS
// ***************************************************************************/


//=============================================================================
void Rosbag2Service::RosbagStartSrvCallback(
    const std::shared_ptr<rosbag2_service_msg::srv::RosbagStart::Request> req,
    std::shared_ptr<rosbag2_service_msg::srv::RosbagStart::Response> response)
//=============================================================================
{
    // Obtain filepath
    //OBTAIN FOLDERS PATH FROM SERVICE

    std::string current_uri = req->path;
    RCLCPP_INFO(nh_->get_logger(), "Starting rosbag file %s", 
        current_uri.c_str());

    //Set stopping variable
    stop_writers_ = false;
    std::map<std::string,std::vector<std::string>> topics_types = 
        nh_->get_topic_names_and_types();

    // start rosbag
    try{
        writer_ = std::make_unique<rosbag2_cpp::Writer>();
        writer_->open(current_uri);
    }
    catch(std::exception &e)
    {
        RCLCPP_ERROR(nh_->get_logger(), e.what());
        response->result = false;
        return;
    }
    
    //Obtain the topics to store
    std::vector<std::string> topics = parameters.topics.as_string_array();
    for (std::vector<std::string>::iterator t=topics.begin(); 
        t!=topics.end(); ++t) 
    {
        RCLCPP_DEBUG(nh_->get_logger(), "Capturing topic %s", t->c_str());

        // Find the type of such topic
        auto pos = topics_types.find(*t);
        if (pos == topics_types.end()) {
            RCLCPP_WARN(nh_->get_logger(), "Topic %s not found", t->c_str());
        } 
        else {
            std::vector<std::string> types = pos->second;
            if (types.size() == 1){
                RCLCPP_DEBUG(nh_->get_logger(), "Type is %s", types[0].c_str());
                //Create subscribers
                std::function<void(
                    std::shared_ptr<rclcpp::SerializedMessage> msg)> fcn = 
                    std::bind(&Rosbag2Service::topic_callback, this, 
                    std::placeholders::_1, *t, types[0]);
                subs_vector_.push_back(
                    nh_->create_generic_subscription(*t, types[0], 10, fcn));
            }
            else{
                RCLCPP_WARN(nh_->get_logger(), 
                    "Found more than one type for the same topic, "
                    "abort capturing this topic");
            }
        }
    }
    // Set up response
    response->result = true;
}

//=============================================================================
void Rosbag2Service::RosbagStopSrvCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
//=============================================================================

{
    RCLCPP_INFO(nh_->get_logger(), "Closing rosbag");
    stop_writers_ = true;
    writer_->close();
    for (std::vector<rclcpp::GenericSubscription::SharedPtr>::iterator t=
        subs_vector_.begin(); t!=subs_vector_.end(); ++t) 
    {
        //Remove subscribers
        t->reset();
    }
    subs_vector_.clear();
    response->success = true;
}

//=============================================================================
void Rosbag2Service::RosbagSaveSrvCallback(
    const std::shared_ptr<rosbag2_service_msg::srv::RosbagSave::Request> req,
    std::shared_ptr<rosbag2_service_msg::srv::RosbagSave::Response> response)
//=============================================================================

{
    // Initialize
    bool did_compress = false;
    // Create compressor
    auto zstd_compressor = rosbag2_compression_zstd::ZstdCompressor{};
    // Obtain filepath
    std::string origin_uri = req->path;
    std::string destination_uri = req->compressed_path;
    // Compress process
    RCLCPP_INFO(nh_->get_logger(), "Compressing rosbag file %s", 
        origin_uri.c_str());
    for (const auto & entry : std::filesystem::directory_iterator(origin_uri)){
        auto path_entry = entry.path();
        auto extension = path_entry.extension();
        if(extension.compare(".db3") == 0){
            did_compress = true;
            std::string compressed_path_uri =  
                zstd_compressor.compress_uri(path_entry); 
        }
    // Clean process
    if (did_compress){
        RCLCPP_INFO(nh_->get_logger(), "Moving compressed files to %s", 
                destination_uri.c_str());
        std::filesystem::copy(origin_uri, destination_uri);
        for (const auto & entry : std::filesystem::directory_iterator(destination_uri)) {
            auto path_entry = entry.path();
            auto extension = path_entry.extension();
            if(extension.compare(".db3") == 0){
                std::filesystem::remove(path_entry);
            }
        }
    }
    else{
        RCLCPP_WARN(nh_->get_logger(), "Nothing compressed, aborting");
    }
        
    }
    response->result = true;
}

// /***************************************************************************
//                 TOPIC CALLBACK FUNCTIONS
// ***************************************************************************/
//================================================
void Rosbag2Service::topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg,
    std::string topic_name, std::string topic_type)
//=============================================================================
{
    if(!stop_writers_) {
        rclcpp::Time time_stamp = nh_->now();
        writer_->write(msg, topic_name, topic_type, time_stamp);
    }
}

// /***************************************************************************
//                 DYNAMIC RECONFIGURATION FUNCTIONS
// ***************************************************************************/


//=============================================================================
rcl_interfaces::msg::SetParametersResult Rosbag2Service::dyn_reconf_callback
    (const std::vector<rclcpp::Parameter> & parameters)
//=============================================================================
{
    RCLCPP_INFO(nh_->get_logger(), "Parameter change request");
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = "The reason could not be allowed";
    for (const auto & parameter : parameters){
        try{
            if (parameter.get_name() == "autorun" && parameter.get_value<bool>() 
                != this->parameters.autorun.get_value<bool>()) {
                result.successful = true;
                result.reason = "Parameter change accepted";
                RCLCPP_INFO(nh_->get_logger(), "Parameter change accepted '%s'",
                    parameter.get_name().c_str());
                this->parameters.autorun = parameter;
                RCLCPP_INFO(nh_->get_logger(),"Parameter '%s' changed",
                    parameter.get_name().c_str());
            }
            else{
                throw std::invalid_argument(
                    "Parameter not found or not available to reconfigure");
            }
            
        }
        catch (const std::invalid_argument& e){
            RCLCPP_WARN(nh_->get_logger(),"Error changing parameter: '%s'",
                e.what());
        }
        catch(std::exception & e){
            RCLCPP_WARN(nh_->get_logger(),e.what());
            result.reason = e.what();
            RCLCPP_INFO(nh_->get_logger(),"Parameter change cancelled '%s'",
                parameter.get_name().c_str());
        }
        catch (...){
            RCLCPP_WARN(nh_->get_logger(),"Parameter change cancelled '%s'", 
                parameter.get_name().c_str());
        }
    }
    return result;
}
} //namespace rosbag2_service
