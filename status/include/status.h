#include<ros/ros.h>

#include<status/status_msg.h>
#include<tf/tf.h>
#include<tf2_msgs/TFMessage.h>
#include<lidar_detection/lidarALL.h>
#include<morai_msgs/GetTrafficLightStatus.h>

#define LiDAR_NUM 6

enum Statusnum
{
    NM, // No Mission
    RC, // Rubber Cone
    SO, // Static Obstacle
    TL, // Traffic Light
    RA, // RoundAbout
    MO, // Moving Obstacle
    WD, // Wait for Detection
    OD, // Obstacle Detected
    RR, // Rubbercone Ready
    RAR // RoundAbout Ready
};

enum Trafficnum
{
    R = 1,      // Red
    G = 16,     // Green
    Y = 4,      // Yellow
    LG = 33,    // Left Green
    RY = 5      // Red Yellow
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

    int lane_num;
    int mission;
    int mission_2and5;

    double x;
    double y;
    double rate = 50;
    double start_time;

    int    object_count;
    double object;

    bool   rubber_cone_state;
    bool   round_about_state;

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
    int     judge_LiDAR_detected(double roi);
    int     judge_LiDAR_number(double roi);
    int     judge_mission();
    int     judge_rubber_cone();
    int     judge_mission_2and5();
    int     judge_lane_num();
    int     judge_round_about();
    bool    judge_traffic_light();
    bool    judge_moving_obstacle();
    void    process();
};