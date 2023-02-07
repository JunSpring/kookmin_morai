#include "Scan2PCL.h"

Scan2PCL::Scan2PCL()
{
    sub = nh.subscribe("/scan", 1, &Scan2PCL::Callback, this); //LaserScan Subscribe
    sub_status = nh.subscribe("/status", 1, &Scan2PCL::Callback_status, this); // PCL2 Subscribe
    pub = nh.advertise<sensor_msgs::PointCloud2>("/pcl2", 1); //PCL2 Publish
    pub_all = nh.advertise<sensor_msgs::PointCloud2>("/pcl2_all", 1); //PCL2 ALL
}

void Scan2PCL::Callback_status(const status::status_msg msg)
{
    status = msg.status;
}

void Scan2PCL::Callback(const sensor_msgs::LaserScan laser_msg)
{
    cloud.clear();
    
    pcl::PointCloud<pcl::PointXYZ> cloud_all;
    sensor_msgs::PointCloud2 cloud2_all;
    
    laser_ranges = laser_msg.ranges;
    range_size = laser_ranges.size();

    if (status == SO || status == MO || status == WD)
    {
        max_range = 2.0;
        ROS_INFO("max range 2.0");
    }

    else 
    {
        max_range = 1.4;
        ROS_INFO("max range 1.4");
    }

    // ROI
    for(size_t i=0; i<range_size; i++)
    {
        if(laser_ranges[i] > min_range && laser_ranges[i] < max_range) // ROI range
        { 
            angle = laser_msg.angle_min + 360.0 * (float(i) / float(range_size));
            if(90.0 <= angle && angle <= 270.0) // ROI angle
            {
                x = (1) * std::get<0>(coordinate);
                y = (-1) * std::get<1>(coordinate);
                coordinate = coordinate_calc(laser_ranges[i], angle); 
                cloud.push_back(pcl::PointXYZ(x, y, 0));
            }
        }
    }   

    cloud2 = toCloud2(cloud);
    pub.publish(cloud2);

    // ROI with no angle
    for(size_t i=0; i<range_size; i++)
    {
        if(laser_ranges[i] > min_range && laser_ranges[i] < 2.5) // ROI range
        { 
            angle = laser_msg.angle_min + 360.0 * (float(i) / float(range_size));
            x = (1) * std::get<0>(coordinate);
            y = (-1) * std::get<1>(coordinate);
            coordinate = coordinate_calc(laser_ranges[i], angle); 
            cloud_all.push_back(pcl::PointXYZ(x, y, 0));
            
        }
    }  

    // publish ROI with no angle
    // cloud2_all = toCloud2(cloud_all);
    // pub_all.publish(cloud2_all);

    //ros::Rate loop_rate(5);
    //loop_rate.sleep();
}

// Point Cloud coordinate calculation
std::tuple<float, float> Scan2PCL::coordinate_calc(float range, float angle)
{
    float x,y;
    x = range * sin(angle * M_PI / 180);
    y = range * cos(angle * M_PI / 180);

    return std::make_tuple(x,y);
}

// PointCloud to PointCloud2 Sensor Messages
sensor_msgs::PointCloud2 Scan2PCL::toCloud2(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(cloud, cloud2);
    cloud2.header.frame_id = "map";
    return cloud2;
}