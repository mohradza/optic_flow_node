#ifndef OPTIC_FLOW_NODE_H
#define OPTIC_FLOW_NODE_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <iostream>
#include <random>

#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif
#include <boost/circular_buffer.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/video/tracking.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

using namespace cv_bridge;
using namespace cv;

namespace wf_of{

class OpticFlowNode{
    public:
        OpticFlowNode(const ros::NodeHandle &node_handle,
                          const ros::NodeHandle &private_node_handle);
        ~OpticFlowNode() = default;

        void init();

        // FUNCTIONS
        //void configCb(Config &config, uint32_t level)
        void imageCb(const sensor_msgs::ImageConstPtr& image_msg);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        std::string node_name_{"node_name"};

        ros::Subscriber image_sub_;

        bool debug_;


};

}

#endif
