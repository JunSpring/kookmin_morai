#include<ros/ros.h>

#include<status/status_msg.h>
#include<tf/tf.h>
#include<tf2_msgs/TFMessage.h>
#include<lidar_detection/lidarALL.h>
#include<morai_msgs/GetTrafficLightStatus.h>

enum Statusnum
{
    NM, // No Mission
    RC, // Rubber Cone
    SO, // Static Obstacle
    TL, // Traffic Light
    RA, // RoundAbout
    MO, // Moving Obstacle
};

enum Trafficnum
{
    R = 1, // Red
    G = 16, // Green
    Y = 4, // Yellow
    LG = 33, // Left Green
    RY = 5, // Red Yellow
};

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

    status::status_msg status_msg;
    
    std::string     traffic_light_index;
    int             traffic_light_status;

    double x;
    double y;
    double rate = 10;

    LiDAR LiDARS[6];

    // Publish
    ros::Publisher status_pub;

    // Subscribe
    ros::Subscriber tf_sub;
    ros::Subscriber lidar_sub;
    ros::Subscriber tl_sub;

    // Callback
    void tfcallback(const tf2_msgs::TFMessage& msg);
    void lidarcallback(const lidar_detection::lidarALL& msg);
    void tlcallback(const morai_msgs::GetTrafficLightStatus& msg);

    // Function
    int     judge_mission();
    int     judge_mission_2and5();
    bool    judge_traffic_light();
    void    process();
};