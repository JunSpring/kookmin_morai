#include "rubber_cone.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Rubber_Cone");
    RubberCone rubber_cone;
    ROS_INFO("Rubber Cone Center Detector Created");
    ros::spin();
}