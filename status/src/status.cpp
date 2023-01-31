#include "status.h"

Status::Status()
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Publish
    status_pub = nh.advertise<status::status_msg>("/status", 10);

    // Subscribe
    tf_sub      = nh.subscribe("/tf", 10, &Status::tfcallback, this);
    lidar_sub   = nh.subscribe("/lidar_info", 10, &Status::lidarcallback, this);
}

void Status::tfcallback(const tf2_msgs::TFMessage& msg)
{
    x = msg.transforms[0].transform.translation.x;
    y = msg.transforms[0].transform.translation.y;
}

void Status::lidarcallback(const lidar_detection::lidarALL& msg)
{
    LiDARS[0].position_x    = msg.position_x0;
    LiDARS[0].position_y    = msg.position_y0;
    LiDARS[0].lidar_update  = msg.lidar0_update;

    LiDARS[1].position_x    = msg.position_x1;
    LiDARS[1].position_y    = msg.position_y1;
    LiDARS[1].lidar_update  = msg.lidar1_update;

    LiDARS[2].position_x    = msg.position_x2;
    LiDARS[2].position_y    = msg.position_y2;
    LiDARS[2].lidar_update  = msg.lidar2_update;

    LiDARS[3].position_x    = msg.position_x3;
    LiDARS[3].position_y    = msg.position_y3;
    LiDARS[3].lidar_update  = msg.lidar3_update;

    LiDARS[4].position_x    = msg.position_x4;
    LiDARS[4].position_y    = msg.position_y4;
    LiDARS[4].lidar_update  = msg.lidar4_update;

    LiDARS[5].position_x    = msg.position_x5;
    LiDARS[5].position_y    = msg.position_y5;
    LiDARS[5].lidar_update  = msg.lidar5_update;
}

int Status::judge_mission()
{
    if(x < -12)
    {
        return 1;
    }
    else if(-11 < x && x < -5 && y < -5)
    {
        return judge_mission_2and5();
    }
    else if(4.8 < x && x < 8.6 && -2 < y && y < 2)
    {
        return 3;
    }
    else if(11 < x && x < 14.7 && -2 < y && y < 2)
    {
        return 4;
    }
    else if(1.8 < x && x < 5.75 && 4 < y && y < 5.7)
    {
        return judge_mission_2and5();
    }
    return 0;
}

int Status::judge_mission_2and5()
{
    return 2;
}

void Status::process()
{
    status_num.status = judge_mission();
    status_pub.publish(status_num);

    switch(status_num.status)
    {
    case 0:
        ROS_INFO("status : no mission driving\t\t\tnumber : 0");
        break;
    case 1:
        ROS_INFO("status : rubber cone mission driving\t\tnumber : 1");
        break;
    case 2:
        ROS_INFO("status : static obstacle mission driving\tnumber : 2");
        break;
    case 3:
        ROS_INFO("status : traffic light mission driving\t  \tnumber : 3");
        break;
    case 4:
        ROS_INFO("status : roundabout mission driving\t\tnumber : 4");
        break;
    case 5:
        ROS_INFO("status : moving obstacle mission driving\tnumber : 5");
        break;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "status");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    Status s;

    ros::Rate loop_rate(s.rate);
    while(ros::ok())
    {
        s.process();
        loop_rate.sleep();
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}