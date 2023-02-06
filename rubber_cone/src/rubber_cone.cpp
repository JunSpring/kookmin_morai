#include "rubber_cone.h"

struct dist_sort {
    bool operator() (std::tuple<float, float, float> dist1, std::tuple<float, float, float> dist2) {return (std::get<2>(dist1) < std::get<2>(dist2));}
} DistSort;

RubberCone::RubberCone()
{
    sub = nh.subscribe("/lidar_info", 1, &RubberCone::Callback, this);
    pub_center = nh.advertise<rubber_cone::rubber_cone_msg>("/cone_center", 1); 
    markerPub = nh.advertise<visualization_msgs::MarkerArray>("/center_marker", 1); 
}

void RubberCone::publish_center(float center_x, float center_y) 
{
    cone_msg.center_x = center_x;
    cone_msg.center_y = center_y;
    center_update = true;
}

void RubberCone::dist_calc(float position_x, float position_y, bool update)
{
    float dist;

    if (update == true)
    {
        dist = std::sqrt(std::pow(position_x, 2) + std::pow(position_y, 2));

        if(position_x < 0) dist_left.push_back(std::make_tuple(position_x, position_y, dist));
        else dist_right.push_back(std::make_tuple(position_x, position_y, dist));
    }

}

void RubberCone::Callback(const lidar_detection::lidarALL& lidar_msg)
{
    float min_dist1_x, min_dist1_y, min_dist2_x, min_dist2_y;
    float center_x, center_y, center_angle;
    visualization_msgs::MarkerArray Markers;
    center_update = false;

    dist_calc(lidar_msg.position_x0, lidar_msg.position_y0, lidar_msg.lidar0_update);
    dist_calc(lidar_msg.position_x1, lidar_msg.position_y1, lidar_msg.lidar1_update);
    dist_calc(lidar_msg.position_x2, lidar_msg.position_y2, lidar_msg.lidar2_update);
    dist_calc(lidar_msg.position_x3, lidar_msg.position_y3, lidar_msg.lidar3_update);
    dist_calc(lidar_msg.position_x4, lidar_msg.position_y4, lidar_msg.lidar4_update);
    dist_calc(lidar_msg.position_x5, lidar_msg.position_y5, lidar_msg.lidar5_update);

    sort(dist_left.begin(), dist_left.end(), DistSort);
    sort(dist_right.begin(), dist_right.end(), DistSort);

    if (dist_left.size() >= 1 && dist_right.size() >= 1)
    {
        min_dist1_x = std::get<0>(dist_left[0]);
        min_dist1_y = std::get<1>(dist_left[0]);
        min_dist2_x = std::get<0>(dist_right[0]);
        min_dist2_y = std::get<1>(dist_right[0]);
        center_x = (min_dist1_x + min_dist2_x)/2;
        center_y = (min_dist1_y + min_dist2_y)/2;

        ROS_INFO("Left & Right Detected");
    }

    else if (dist_left.size() < 1 && dist_right.size() >= 1)
    {
        min_dist1_x = std::get<0>(dist_right[0]);
        min_dist1_y = std::get<1>(dist_right[0]);

        center_x = min_dist1_x - 0.6;
        center_y = min_dist1_y;

        ROS_INFO("Only Right Detected");
    }

    else if (dist_right.size() < 1 && dist_left.size() >= 1)
    {
        min_dist1_x = std::get<0>(dist_left[0]);
        min_dist1_y = std::get<1>(dist_left[0]);

        center_x = min_dist1_x + 0.6;
        center_y = min_dist1_y;

        ROS_INFO("Only Left Detected");
    }

    center_angle = std::atan2(center_x, center_y) * 180 / M_PI;

    if ((dist_left.size() + dist_right.size())  > 0)
    {
        visualization_msgs::Marker m;
        
        m.type = visualization_msgs::Marker::CYLINDER;
        m.header.frame_id = "map";
        m.scale.x = 0.3;
        m.scale.y = 0.3;
        m.scale.z = 0.3;
        m.color.a = 1.0;
        m.color.r = 0;
        m.color.g = 0;
        m.color.b = 1;

        m.pose.position.x = center_x;
        m.pose.position.y = center_y;
        m.pose.position.z = 0.0;
        
    
        if (abs(m.pose.position.x) > 0.01 && abs(m.pose.position.y) > 0.01 && abs(m.pose.position.x) < MAX_DIST && abs(m.pose.position.y) < MAX_DIST)
        {
            m.action = visualization_msgs::Marker::ADD;
            Markers.markers.push_back(m);     
            publish_center(m.pose.position.x, m.pose.position.y);
            // ROS_INFO("x: %f y: %f", m.pose.position.x, m.pose.position.y);
        }

        else
        {
            m.action = visualization_msgs::Marker::DELETE;
            Markers.markers.push_back(m); 
        }
    }

    cone_msg.center_angle = center_angle;
    cone_msg.center_update = center_update;
    pub_center.publish(cone_msg);
    markerPub.publish(Markers);
    dist_left.clear();
    dist_right.clear();
}