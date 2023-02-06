#define _USE_MATH_DEFINES

#include<ros/ros.h>
#include<iostream>
#include<math.h>
#include<cmath>
#include<vector>

#include<nav_msgs/Path.h>

#include<geometry_msgs/PointStamped.h>

#include<visualization_msgs/MarkerArray.h>

#include<std_msgs/Float64.h>

#include<tf/tf.h>
#include<tf2_msgs/TFMessage.h>

#include<sensor_msgs/Imu.h>

#include<status/status_msg.h>

// #include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

// // DWA include files
// #include <tf/transform_listener.h>
// #include <costmap_2d/costmap_2d_ros.h>
// #include <dwa_local_planner/dwa_planner_ros.h>

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

class local_path
{
public:
    local_path();
    
    int num;
    int status;
    int lane_num;

    bool mission3_go;
    bool mission5_go;
    bool flag;

    bool is_look_foward_point;
    double L, VL;
    double vehicle_roll;
    double vehicle_pitch;
    double vehicle_yaw;
    double max_lfd, min_lfd;
    double steering_angle;

    double rate = 50;
    double offset = 335.7699221348003;
    double steer_offset = 19.4799995422;

    double bx;
    double by;
    double bz;
    double bw;

    nav_msgs::Path path;
    nav_msgs::Path tracking_path;

    geometry_msgs::PoseStamped pose;

    std_msgs::Float64 speed;
    std_msgs::Float64 position;

    tf2_msgs::TFMessage tfmsg;

    //Publish
    ros::Publisher speed_pub;
    ros::Publisher position_pub;
    ros::Publisher path_pub;
    ros::Publisher marker_lfd;
    ros::Publisher local_goal_pub;

    // Subscriber
    ros::Subscriber path_sub;
    ros::Subscriber tf_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber status_sub;

    // Callback
    void pathcallback(const nav_msgs::Path& msg);
    void tfcallback(const tf2_msgs::TFMessage& msg);
    void imucallback(const sensor_msgs::Imu& msg);
    void statuscallback(const status::status_msg& msg);
    
    // Function
    double pure_pursuit();
    double stanley();
    double velocity();
    double nomalize_angle(double);
    void path_tracking();
    void mission();
    void process();

    // Driving Function
    void go(double speed_input);
    void go_slow(double speed_input);
    void stop();
};