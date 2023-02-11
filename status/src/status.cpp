#include "status.h"

Status::Status()
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    status_msg.lane_num = 2;
    start_time = 0;
    mission = WD;

    rubber_cone_state = false;
    round_about_state = false;

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

int Status::judge_LiDAR_detected(double roi)
{
    int LiDAR_index;
    double least = 10000;
    double least_index = -1;

    for(LiDAR_index = 0; LiDAR_index < LiDAR_NUM; LiDAR_index++)
    {
        if(-1*roi < LiDARS[LiDAR_index].position_x && LiDARS[LiDAR_index].position_x < roi && LiDARS[LiDAR_index].lidar_update)
        {
            if(pow(LiDARS[LiDAR_index].position_x, 2) + pow(LiDARS[LiDAR_index].position_y, 2) < least)
            {
                least_index = LiDAR_index;
                least = pow(LiDARS[LiDAR_index].position_x, 2) + pow(LiDARS[LiDAR_index].position_y, 2);
            }
        }
    }
    
    return least_index;
}

int Status::judge_LiDAR_number(double roi)
{
    int number = 0;
    int LiDAR_index;

    for(LiDAR_index = 0; LiDAR_index < LiDAR_NUM; LiDAR_index++)
    {
        if(-1*roi < LiDARS[LiDAR_index].position_x && LiDARS[LiDAR_index].position_x < roi && LiDARS[LiDAR_index].lidar_update)
        {
            number++;
        }
    }

    return number;
}

int Status::judge_mission()
{
    if(x < -12.5)
    {
        return judge_rubber_cone();
    }
    else if(-12.4 < x && x < -5 && y < -4.0)
    {
        mission_2and5 = 2;
        return judge_mission_2and5();
    }
    else if(4.8 < x && x < 8.6 && -2.54 < y && y < 2.15)
    {
        return TL;
    }
    else if(11 < x && x < 14.7 && -2.15 < y && y < 2.75)
    {
        return judge_round_about();
    }
    else if(1.8 < x && x < 6.75 && 4.15 < y && y < 5.85)
    {
        mission_2and5 = 5;
        return judge_mission_2and5();
    }
    return NM;
}

int Status::judge_rubber_cone()
{
    int LiDAR_number = judge_LiDAR_number(0.8);

    if(!rubber_cone_state)
    {
        if(LiDAR_number >= 3)
        {
            rubber_cone_state = true;
            return RC;
        }
        return RR;
    }
    else
    {
        if(LiDAR_number < 3)
            return RD;
        return RC;
    }
}

int Status::judge_mission_2and5()
{
    if(mission == SO || mission == MO)
        return mission;
    
    
    int LiDAR_index = judge_LiDAR_detected(0.8);

    if(LiDAR_index == -1 && start_time == 0)
        return WD;
    else if(LiDAR_index != -1 && start_time == 0)
    {
        start_time = ros::Time::now().toSec();
        object = LiDARS[LiDAR_index].position_x;
        object_count = 0;
    }
    else if(ros::Time::now().toSec() - start_time < 1)
    {
        if(LiDAR_index != -1 && abs(object - LiDARS[LiDAR_index].position_x) > 0.02)
        {
            object_count++;
            object = LiDARS[LiDAR_index].position_x;
        }
        
        if(LiDAR_index != -1)
            ROS_INFO("%f", object - LiDARS[LiDAR_index].position_x);
    }
    else if(start_time != 0)
    {
        ROS_INFO("%d", object_count);
        if(object_count >= 3)
            mission = MO;
        else
            mission = SO;
        start_time = 0;
        return mission;
    }

    return OD;
}

int Status::judge_lane_num()
{
    int LiDAR_index = judge_LiDAR_detected(0.1);
    static double real_data_x = -1;
    static double real_data_y = -1;

    int lane = status_msg.lane_num;

    if(LiDAR_index != -1 && ros::Time::now().toSec() - start_time > 1.0)
    {
        real_data_x = LiDARS[LiDAR_index].position_x;
        real_data_y = LiDARS[LiDAR_index].position_y;
    }
    
    if(0 < real_data_y && real_data_y < 2.0 && ros::Time::now().toSec() - start_time > 1.0)
    {
        ROS_INFO("%f", real_data_y);
        if(status_msg.lane_num == 2)
        {
            if(mission_2and5 == 2)
                lane = 12;
            else
                lane = 15;
        }
        else
            lane = 2;

        real_data_x = -1;
        real_data_y = -1;
        start_time = ros::Time::now().toSec();
    }

    return lane;
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

int Status::judge_round_about()
{
    int LiDAR_index = judge_LiDAR_detected(1.4);

    if(!round_about_state)
    {
        if(y > 2.15)
        {
            return RA;
        }
        else
        {
            if(LiDAR_index == -1)
            {
                return RAR;
            }
            else
            {
                if(LiDARS[LiDAR_index].position_x > 0)
                {
                    round_about_state = true;
                    return RA;
                }
                else
                    return RAR;
            }
        }
    }
    else
    {
        if(LiDAR_index == -1)
            return RA;
        else
        {
            if(pow(LiDARS[LiDAR_index].position_x, 2) + pow(LiDARS[LiDAR_index].position_y, 2) < 1.0)
            {
                return RAR;
            }
            else
                return RA;
        }
    }
}

bool Status::judge_moving_obstacle()
{
    int LiDAR_index = judge_LiDAR_detected(0.8);
    static double real_data_x = -1;
    static double real_data_y = -1;

    if(LiDAR_index != -1)
    {
        real_data_x = LiDARS[LiDAR_index].position_x;
        real_data_y = LiDARS[LiDAR_index].position_y;
    }
    
    if(real_data_x < 0.4)
        return false;
    return true;
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
        status_msg.lane_num = 2;
        rubber_cone_state = false;
        round_about_state = false;
        ROS_INFO("status : no mission driving\t\t\tnumber : 0");
        break;
    case RC:
        ROS_INFO("status : rubber cone mission driving\t\tnumber : 1");
        break;
    case SO:
        status_msg.lane_num = judge_lane_num();
        ROS_INFO("status : static obstacle mission driving\tnumber : 2\tlane number : %d", status_msg.lane_num);
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
    default:
        ROS_INFO("%d", status_msg.status);
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