#include<ros/ros.h>
#include<iostream>
#include<math.h>
#include<cmath>
#include<vector>

#include<nav_msgs/Path.h>

#include<geometry_msgs/PointStamped.h>

#include<morai_msgs/CtrlCmd.h>

#include<tf/tf.h>
#include<tf2_msgs/TFMessage.h>

class local_path{
public:
    local_path();
    
    int speed;
    int num;
    bool flag;
    double vehicle_roll;
    double vehicle_pitch;
    double vehicle_yaw;
    double rate = 10;

    nav_msgs::Path path;
    nav_msgs::Path tracking_path;

    geometry_msgs::PoseStamped pose;

    morai_msgs::CtrlCmd cmd_vel;

    tf2_msgs::TFMessage tfmsg;

    //Publish
    ros::Publisher cmd_pub;
    ros::Publisher path_pub;

    // Subscriber
    ros::Subscriber path_sub;
    ros::Subscriber tf_sub;

    // Callback
    void pathcallback(const nav_msgs::Path& msg);
    void tfcallback(const tf2_msgs::TFMessage& msg);
    
    //Function
    double stanley();
    double velocity();
    void path_tracking();
    void process();
};