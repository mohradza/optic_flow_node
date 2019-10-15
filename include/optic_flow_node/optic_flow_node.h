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
#include <boost/thread.hpp>
#include <boost/circular_buffer.hpp>
#include <std_msgs/Float32MultiArray.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/video/tracking.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <optic_flow_node/OpticFlowNodeConfig.h>


using namespace cv_bridge;
using namespace cv;
using namespace std;

namespace wf_of{

class OpticFlowNode{
    public:
        OpticFlowNode(const ros::NodeHandle &node_handle,
                          const ros::NodeHandle &private_node_handle);
        ~OpticFlowNode() = default;

        void init();

        // FUNCTIONS
        void imageCb(const sensor_msgs::ImageConstPtr& image_msg);


    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        std::string node_name_{"node_name"};

        boost::mutex connect_mutex_;
        boost::recursive_mutex config_mutex_;
        typedef optic_flow_node::OpticFlowNodeConfig Config;
        typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
        boost::shared_ptr<ReconfigureServer> reconfigure_server_;
        Config config_;
        void configCb(Config &config, uint32_t level);

        ros::Subscriber sub_image_;
        ros::Publisher pub_tang_flow_;

        std::string OPENCV_WINDOW;
        bool debug_;
        std::vector<Point2f> points2track_;
        std::vector<Point2f> newpoints_;
        std::vector<float> u_flow_;
        std::vector<float> v_flow_;
        std::vector<float> tang_flow_;
        std::vector<float> ave_tang_flow_;
        std::vector<float> gamma_vector_;

        int image_center_x_;
        int image_center_y_;
        double inner_ring_radius_;
        int num_ring_points_;
        int num_rings_;
        int ring_dr_;
        Mat prev_grey_image_;
        bool init_;
        int win_size_;
        double pixel_scale_;
        float dt_;

        ros::Time image_timestamp_;
        ros::Time last_image_timestamp_;

};

}

#endif
