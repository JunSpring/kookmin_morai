#include "status.h"

Status::Status()
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    lane_num = 2;
    start_time = 0;
    mission = WD;

    // Publish
    status_pub = nh.advertise<status::status_msg>("/status", 10);

    // Subscribe
    tf_sub      = nh.subscribe("/tf", 10, &Status::tfcallback, this);
    lidar_sub   = nh.subscribe("/lidar_info", 10, &Status::lidarcallback, this);
    tl_sub      = nh.subscribe("/GetTrafficLightStatus", 10, &Status::tlcallback, this);
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

void Status::tlcallback(const morai_msgs::GetTrafficLightStatus& msg)
{
    traffic_light_index     = msg.trafficLightIndex;
    traffic_light_status    = msg.trafficLightStatus;
}

int Status::judge_mission()
{
    if(x < -12)
    {
        return RC;
    }
    else if(-11 < x && x < -5 && y < -5.15)
    {
        return judge_mission_2and5();
    }
    else if(4.8 < x && x < 8.6 && -2.54 < y && y < 2.15)
    {
        return TL;
    }
    else if(11 < x && x < 14.7 && -2.15 < y && y < 2.15)
    {
        return RA;
    }
    else if(1.8 < x && x < 5.75 && 4.15 < y && y < 5.85)
    {
        return judge_mission_2and5();
    }
    return NM;
}

int Status::judge_mission_2and5()
{
    if(mission == SO || mission == MO)
        return mission;
    
    bool LiDAR_detected = false;
    int  LiDAR_index;

    for(LiDAR_index = 0; LiDAR_index < LiDAR_NUM; LiDAR_index++)
    {
        if(-0.8 < LiDARS[LiDAR_index].position_x && LiDARS[LiDAR_index].position_x < 0.8)
        {
            LiDAR_detected = true;
            break;
        }
    }

    if(LiDAR_detected && start_time == 0)
    {
        start_time = ros::Time::now().toSec();
        object = LiDARS[LiDAR_index].position_x;
        object_count = 0;
    }
    else if(ros::Time::now().toSec() - start_time < 2)
    {
        if(LiDAR_detected && abs(object - LiDARS[LiDAR_index].position_x) > 0.1)
            object_count++;
    }
    else if(start_time != 0 && ros::Time::now().toSec() - start_time >= 2)
    {
        if(object_count >= 10)
            mission = MO;
        else
            mission = SO;
        return mission;
    }

    return WD;
}

int Status::judge_lane_num()
{
    return 2;
}

bool Status::judge_traffic_light()
{
    if(traffic_light_index == "SN000002")
    {
        if(traffic_light_status == G)
            return true;
        return false;
    }
    return true;
}

bool Status::judge_moving_obstacle()
{

}

void Status::process()
{
    status_msg.status = judge_mission();
    status_pub.publish(status_msg);

    switch(status_msg.status)
    {
    case NM:
        start_time = 0;
        mission = WD;
        ROS_INFO("status : no mission driving\t\t\tnumber : 0");
        break;
    case RC:
        ROS_INFO("status : rubber cone mission driving\t\tnumber : 1");
        break;
    case SO:
        status_msg.lane_num = judge_lane_num();
        ROS_INFO("status : static obstacle mission driving\tnumber : 2\tlane number : %d", lane_num);
        break;
    case TL:
        status_msg.mission3_go = judge_traffic_light();
        if(status_msg.mission3_go)
            ROS_INFO("status : traffic light mission driving\t\tnumber : 3\ttraffic light : Green");
        else
            ROS_INFO("status : traffic light mission driving\t\tnumber : 3\ttraffic light : Else");
        break;
    case RA:
        ROS_INFO("status : roundabout mission driving\t\tnumber : 4");
        break;
    case MO:
        status_msg.mission5_go = judge_moving_obstacle();
        if(status_msg.mission5_go)
            ROS_INFO("status : moving obstacle mission driving\tnumber : 5\tdriving mode : Go");
        else
            ROS_INFO("status : moving obstacle mission driving\tnumber : 5\tdriving mode : Stop");
        break;
    case WD:
        ROS_INFO("status : Wait for Detection\t\t\tnumber : 6");
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