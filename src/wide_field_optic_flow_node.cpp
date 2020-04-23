#include "optic_flow_node/optic_flow_node.h"
#include <string>

int main(int argc, char** argv) {
    std::string node_name = "wide_field_optic_flow_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("wide_field_optic_flow_node");
    ros::NodeHandle nh_private("~");
    wf_of::OpticFlowNode optic_flow_node(nh, nh_private);
    ros::spin();
}
