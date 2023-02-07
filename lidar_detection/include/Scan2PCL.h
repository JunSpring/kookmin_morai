#include "sensor_msgs/PointCloud2.h"
#include "status/status_msg.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"
#include "pcl/point_types.h"
#include "pcl/PCLPointCloud2.h"
#include "pcl/conversions.h"
#include "pcl_conversions/pcl_conversions.h"
#include "math.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <tuple>
#define _USE_MATH_DEFINES

enum Statusnum
{
    NM, // No Mission
    RC, // Rubber Cone
    SO, // Static Obstacle
    TL, // Traffic Light
    RA, // RoundAbout
    MO, // Moving Obstacle
    WD  // Wait for Detection
};

class Scan2PCL
{
public:
    std::tuple<float, float> coordinate_calc(float range, float angle);
    sensor_msgs::PointCloud2 toCloud2(pcl::PointCloud<pcl::PointXYZ> cloud);
    Scan2PCL();
    void Callback(const sensor_msgs::LaserScan msg);
    void Callback_status(const status::status_msg msg);

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Subscriber sub_status;
    ros::Publisher pub;
    ros::Publisher pub_all;

    int status;
    
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 cloud2;
    std::vector<float> laser_ranges;
    int index;
    size_t range_size;
    float max_range;
    float min_range = 0.2;
    float angle, x, y;    
    std::tuple<float, float> coordinate;
};