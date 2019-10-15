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
    /*
    reconfigure_server_.reset(new ReconfigureServer(config_mutex_, pnh_));
    ReconfigureServer::CallbackType f = boost::bind(&OpticFlowNode::configCb, this, _1, _2);
    reconfigure_server_->setCallback(f);
    */
    debug_ = true;
    init_ = true;
    // Set up subscribers and callbacks
    image_sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &OpticFlowNode::imageCb, this);

    // Set up publishers
    //pub_h_scan_reformat_ = nh_.advertise<std_msgs::Float32MultiArray>("horiz_depth_reformat", 10);


    // Import parameters
    nh_.param("/optic_flow_node/image_center_x", image_center_x_, 320);
    nh_.param("/optic_flow_node/image_center_y", image_center_y_, 240);
    nh_.param("/optic_flow_node/inner_ring_radius", inner_ring_radius_, 150.0);
    nh_.param("/optic_flow_node/num_ring_points", num_ring_points_, 30);
    nh_.param("/optic_flow_node/num_rings", num_rings_, 1);
    nh_.param("/optic_flow_node/ring_dr", ring_dr_, 5);

    //ROS_INFO("%d", total_h_scan_points_);

    // Generate the gamma vector
    for(int i = 0; i < num_ring_points_; i++){
        gamma_vector_.push_back((float(i)/float(num_ring_points_-1))*2*M_PI - M_PI);
    }

    // Figure out which points we want to track
    int x = 1;
    int y = 2;
    //std::vector<Point2f> points;
    points2track_.push_back(Point2f((float)image_center_x_, (float)image_center_y_));
    cout << points2track_ << std::endl;
    for(int r = 0; r < num_rings_; r++){
        for(int i = 0; i < num_ring_points_; i++){
           int x = image_center_x_ + int((inner_ring_radius_+float(r)*ring_dr_)*sin(gamma_vector_[i]));
           int y =  image_center_y_ - int((inner_ring_radius_+float(r)*ring_dr_)*cos(gamma_vector_[i]));
           points2track_.push_back(Point2f((float)x, (float)y));
           cout << points2track_[r*num_ring_points_ + i] << std::endl;
        }
    }
    newpoints_ = points2track_;

    // Set up an opencv window for debugging
    OPENCV_WINDOW = "Image Window";

    if(debug_){
      namedWindow(OPENCV_WINDOW);
    }

} // End of init

//void NearnessController::configCb(Config &config, uint32_t level)
//{
//    config_ = config;

    // Controller gains
    //u_k_hb_1_ = config_.forward_speed_k_hb_1;
//}

void OpticFlowNode::imageCb(const sensor_msgs::ImageConstPtr& image_msg){

    // Convert the ros image msg into a cv mat
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
    }

    vector<uchar> status;
    vector<float> err;
    cv::Size winSize(30,30);
    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);

    cv::calcOpticalFlowPyrLK(prev_grey_image_, grey_image, points2track_, newpoints_, status, err, winSize, 3, termcrit, 0, 0.001);

    // Process the CV mat and compute optic optic flow at various pixels
    for(int i= 1; i < num_ring_points_-1; i++){
        cv::line(image_ptr->image, points2track_[i], newpoints_[i], CV_RGB(255,0,0),2);
    }
    prev_grey_image_ = grey_image;

    // Display the image with resulting flow
    if(debug_){
        imshow("window1", image_ptr->image);
        waitKey(3);
    }
}


 // end of class
} // End of namespace nearness
