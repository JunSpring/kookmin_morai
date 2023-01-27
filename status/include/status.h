#include<ros/ros.h>

#include<status/status_msg.h>
#include<tf/tf.h>
#include<tf2_msgs/TFMessage.h>
#include<lidar_detection/lidarALL.h>

struct LiDAR
{
    double  position_x;
    double  position_y;
    bool    lidar_update;
};

class Status
{
public:
    Status();

    status::status_msg status_num;
    
    double x;
    double y;
    double rate = 10;

    LiDAR LiDARS[6];

    // Publish
    ros::Publisher status_pub;

    // Subscribe
    ros::Subscriber tf_sub;
    ros::Subscriber lidar_sub;

    // Callback
    void tfcallback(const tf2_msgs::TFMessage& msg);
    void lidarcallback(const lidar_detection::lidarALL& msg);

    // Function
    int     judge_mission();
    int     judge_mission_2and5();
    void    process();
};