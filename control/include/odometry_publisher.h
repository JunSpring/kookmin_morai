#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

// Subscribe
#include <morai_msgs/EgoVehicleStatus.h>
#include <sensor_msgs/Imu.h>
#include <tf2_msgs/TFMessage.h>

class Odometry
{
public:
    Odometry();

    // variable
    double x;
    double y;
    double th;

    double vx;
    double vy;
    double vth;

    double rate = 10;

    ros::Time current_time;
    ros::Time last_time;

    geometry_msgs::Quaternion odom_quat;

    nav_msgs::Odometry odom;

    // Publish
    ros::Publisher odom_pub; 

    // Subscribe
    ros::Subscriber Ego_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber tf_sub;

    // Callback
    void Egocallback(const morai_msgs::EgoVehicleStatus& msg);
    void imucallback(const sensor_msgs::Imu& msg);
    void tfcallback(const tf2_msgs::TFMessage& msg);

    // Function
    void process(ros::Time ct, ros::Time lt);
};