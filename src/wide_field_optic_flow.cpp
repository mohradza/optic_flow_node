#include <optic_flow_node/optic_flow_node.h>

namespace wf_of{

OpticFlowNode::OpticFlowNode(const ros::NodeHandle &node_handle,
                                       const ros::NodeHandle &private_node_handle)
      :nh_(node_handle),
       pnh_(private_node_handle) {
      this->init();
  }

void OpticFlowNode::init() {
    // Set up dynamic reconfigure

    reconfigure_server_.reset(new ReconfigureServer(config_mutex_, pnh_));
    ReconfigureServer::CallbackType f = boost::bind(&OpticFlowNode::configCb, this, _1, _2);
    reconfigure_server_->setCallback(f);

    init_ = true;
    // Set up subscribers and callbacks
    sub_image_ = nh_.subscribe("/usb_cam/image_raw", 1, &OpticFlowNode::imageCb, this);

    // Set up publishers
    pub_tang_flow_ = nh_.advertise<std_msgs::Float32MultiArray>("tang_optic_flow", 10);

    // Import parameters
    nh_.param("/optic_flow_node/image_center_x", image_center_x_, 325);
    nh_.param("/optic_flow_node/image_center_y", image_center_y_, 220);
    nh_.param("/optic_flow_node/inner_ring_radius", inner_ring_radius_, 150.0);
    nh_.param("/optic_flow_node/num_ring_points", num_ring_points_, 30);
    nh_.param("/optic_flow_node/num_rings", num_rings_, 1);
    nh_.param("/optic_flow_node/ring_dr", ring_dr_, 5);

    // LK parameters
    nh_.param("/optic_flow_node/pyr_window_size", win_size_, 30);
    nh_.param("/optic_flow_node/pixel_scale", pixel_scale_, 15.0);

    // Switches
    nh_.param("/optic_flow_node/enable_debug", debug_, true);

    // Set up an opencv window for debugging
    OPENCV_WINDOW = "Image Window";

    if(debug_){
      namedWindow(OPENCV_WINDOW);
    }

} // End of init

void OpticFlowNode::configCb(Config &config, uint32_t level)
{
    config_ = config;

    image_center_x_ = config.image_center_x;
    image_center_y_ = config.image_center_y;
    inner_ring_radius_ = config.inner_ring_radius;
    ring_dr_ = config.ring_dr;
    num_ring_points_ = config.num_ring_points;
    num_rings_ = config.num_rings;
    win_size_ = config.pyr_window_size;
    pixel_scale_ = config.pixel_scale;

    gamma_vector_.clear();
    for(int i = 0; i < num_ring_points_; i++){
        gamma_vector_.push_back((float(i)/float(num_ring_points_-1))*2*M_PI - M_PI);
    }

    points2track_.clear();
    points2track_.push_back(Point2f((float)image_center_x_, (float)image_center_y_));
    for(int r = 0; r < num_rings_; r++){
        for(int i = 0; i < num_ring_points_; i++){
           int x = image_center_x_ + int((inner_ring_radius_+float(r)*ring_dr_)*sin(gamma_vector_[i]));
           int y =  image_center_y_ - int((inner_ring_radius_+float(r)*ring_dr_)*cos(gamma_vector_[i]));
           points2track_.push_back(Point2f((float)x, (float)y));
        }
    }

}

void OpticFlowNode::imageCb(const sensor_msgs::ImageConstPtr& image_msg){

    // Convert the ros image msg into a cv mat
    image_timestamp_ = image_msg->header.stamp;
    CvImagePtr image_ptr;
    try
    {
        image_ptr = toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }


    cv::Mat grey_image;
    cv::Mat grey_image_overlay;
    cv::cvtColor(image_ptr->image, grey_image, CV_BGR2GRAY);

    if (init_){
        init_ = false;
        prev_grey_image_ = grey_image;
        dt_ = 10000;
    } else {
        dt_ = (image_timestamp_ - last_image_timestamp_).toSec();
    }
    last_image_timestamp_ = image_timestamp_;

    vector<uchar> status;
    vector<float> err;
    cv::Size winSize(win_size_,win_size_);
    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);

    cv::calcOpticalFlowPyrLK(prev_grey_image_, grey_image, points2track_, newpoints_, status, err, winSize, 3, termcrit, 0, 0.001);

    // Process the CV mat and compute optic optic flow at various pixels
    for (int i= 1; i < num_ring_points_*num_rings_; i++){
        cv::line(image_ptr->image, points2track_[i], newpoints_[i], CV_RGB(255,0,0),2);
    }
    prev_grey_image_ = grey_image;

    // Convert flow ring points in u-v coordinates to tangential optic flow
    u_flow_.clear();
    v_flow_.clear();
    for (int i = 0; i < num_ring_points_*num_rings_; i++){
        u_flow_.push_back((points2track_[i].x - newpoints_[i].x)*pixel_scale_/dt_);
        v_flow_.push_back(points2track_[i].y - newpoints_[i].y);
        //ROS_INFO("%f, %f", u_flow_[i], v_flow_[i]);
    }
    // Convert u-v flow to tangential flow
    int index;
    tang_flow_.clear();
    for (int r = 0; r < num_rings_; r++){
        for (int i = 0; i < num_ring_points_; i++){
            index = r*num_ring_points_ + i;
            tang_flow_.push_back(u_flow_[index]*cos(gamma_vector_[i]) + v_flow_[i]*sin(gamma_vector_[i]));
        }
    }

    // Compute the spatial average
    std_msgs::Float32MultiArray tang_flow_msg;
    ave_tang_flow_.clear();
    for (int i = 0; i < num_ring_points_; i++){
        ave_tang_flow_.push_back(tang_flow_[i]/num_rings_);
    }
    tang_flow_msg.data = ave_tang_flow_;

    pub_tang_flow_.publish(tang_flow_msg);

    // Display the image with resulting flow
    if(debug_){
        imshow("window1", image_ptr->image);
        waitKey(3);
    }
}


 // end of class
} // End of namespace nearness
