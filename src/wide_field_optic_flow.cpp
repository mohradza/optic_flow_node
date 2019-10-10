#include <optic_flow_node/optic_flow_node.h>

namespace wf_of{
OpticFlowNode::OpticFlowNode(const ros::NodeHandle &node_handle,
                                       const ros::NodeHandle &private_node_handle)
      :nh_(node_handle),
       pnh_(private_node_handle) {
      this->init();
  }

void NearnessController::init() {
    // Set up dynamic reconfigure
    reconfigure_server_.reset(new ReconfigureServer(config_mutex_, pnh_));
    ReconfigureServer::CallbackType f = boost::bind(&OpticFlowNode::configCb, this, _1, _2);
    reconfigure_server_->setCallback(f);

    debug_ = true;

    // Set up subscribers and callbacks
    image_callback_ = nh_.subscribe("some_image_topic", 1, &OpticFlowNode::imageCb, this);

    // Set up publishers
    pub_h_scan_reformat_ = nh_.advertise<std_msgs::Float32MultiArray>("horiz_depth_reformat", 10);


    // Import parameters
    nh_.param("/nearness_control_node/total_horiz_scan_points", total_h_scan_points_, 1440);
    ROS_INFO("%d", total_h_scan_points_);

} // End of init

void NearnessController::configCb(Config &config, uint32_t level)
{
    config_ = config;

    // Controller gains
    u_k_hb_1_ = config_.forward_speed_k_hb_1;

}

void OpticFlowNode::imageCb(const sensor_msgs::LaserScanPtr h_laserscan_msg){



}


 // end of class
} // End of namespace nearness
