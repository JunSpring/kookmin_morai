#include "control.h"

local_path::local_path()
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.param("num",num,30);

    // Publish
    cmd_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd",10);
    path_pub = nh.advertise<nav_msgs::Path>("tracking_path",10);

    // Subscibe
    path_sub = nh.subscribe("/path",10,&local_path::pathcallback,this);
    tf_sub = nh.subscribe("/tf", 10, &local_path::tfcallback, this);
}

void local_path::pathcallback(const nav_msgs::Path& msg)
{
    path = msg;
    flag = true;
}

void local_path::tfcallback(const tf2_msgs::TFMessage& msg)
{
    tfmsg = msg;

    pose.pose.position.x = msg.transforms[0].transform.translation.x;
    pose.pose.position.y = msg.transforms[0].transform.translation.y;
    pose.pose.position.z = msg.transforms[0].transform.translation.z;
    pose.pose.orientation.x = msg.transforms[0].transform.rotation.x;
    pose.pose.orientation.y = msg.transforms[0].transform.rotation.y;
    pose.pose.orientation.z = msg.transforms[0].transform.rotation.z;
    pose.pose.orientation.w = msg.transforms[0].transform.rotation.z;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";

    tf::Quaternion car_orientation(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    tf::Matrix3x3 matrix(car_orientation);
    matrix.getRPY(vehicle_roll, vehicle_pitch, vehicle_yaw);
}

void local_path::path_tracking()
{
    if(flag == true)
    {
        double least_dist = 10;
        int temp_num;
        int num = this->num;

        for(int i = 0; i<path.poses.size(); i++)
        {
            double dx = pose.pose.position.x - path.poses.at(i).pose.position.x;
            double dy = pose.pose.position.y - path.poses.at(i).pose.position.y;

            double dist = sqrt(dx*dx + dy*dy);
            if(dist<least_dist)
            {
                least_dist = dist;
                temp_num = i;
            }
        }

//        double avoid_index = 0;
//        double short_distance;
//        double center_path,right_path,left_path;

//        if(avoid == true)
//        {
//            for(int i = 0; i<tracking_path.poses.size(); i++)
//            {
//                double dx = obstacle.points.at(0).x - tracking_path.poses.at(i).pose.position.x;
//                double dy = obstacle.points.at(0).y - tracking_path.poses.at(i).pose.position.y;

//                double dis = sqrt(dx*dx + dy*dy);
//                if(short_distance > dis)
//                {
//                    short_distance = dis;
//                    avoid_index = i;
//                }
//            }
//        }

        tracking_path.header.stamp = ros::Time::now();
        tracking_path.header.frame_id = "map";
        tracking_path.poses.clear();

        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header.stamp = ros::Time::now();
        temp_pose.header.frame_id = "map";

        if(temp_num + num <= path.poses.size())
        {
            for(int i = temp_num; i< temp_num + num; i++)
            {
                // if(avoid == true)
                // {
                //     temp_pose.pose.position.x = path.poses.at(i).pose.position.x + 4*sin(vehicle_yaw);
                //     temp_pose.pose.position.y = path.poses.at(i).pose.position.y - 4*cos(vehicle_yaw);
                //     temp_pose.pose.position.z = 0;
                //     tracking_path.poses.push_back(temp_pose);
                // }
                // else
                // {
                    temp_pose.pose.position.x = path.poses.at(i).pose.position.x;
                    temp_pose.pose.position.y = path.poses.at(i).pose.position.y;
                    temp_pose.pose.position.z = 0;
                    tracking_path.poses.push_back(temp_pose);
                // }

//                temp_pose.pose.position.x = path.poses.at(i).pose.position.x - 4*sin(vehicle_yaw);
//                temp_pose.pose.position.y = path.poses.at(i).pose.position.y + 4*cos(vehicle_yaw);
//                temp_pose.pose.position.z = 0;
//                tracking_path1.poses.push_back(temp_pose);
            }
        }
        else
        {
            flag = false;
        }
        path_pub.publish(tracking_path);
    }
    else
    {
        ROS_INFO("DONE");
    }
}

double local_path::velocity()
{
   double vel = 1;

   return vel;
}

double local_path::stanley()
{
    double px,py,cyaw;
    double min_dist = 1e9;
    double min_index = 0;
    double k = 0.5;

    double steering;

    if(tracking_path.poses.size() > 1)
    {
        geometry_msgs::PoseStamped path_pose = tracking_path.poses.at(0);
        geometry_msgs::PoseStamped next_pose = tracking_path.poses.at(1);

        px = next_pose.pose.position.x - path_pose.pose.position.x;
        py = next_pose.pose.position.y - path_pose.pose.position.y;

        cyaw = atan2(py,px);

        double front_x = pose.pose.position.x + 0.1*cos(vehicle_yaw);
        double front_y = pose.pose.position.y + 0.1*sin(vehicle_yaw);

        for(int i = 0; i<tracking_path.poses.size(); i++)
        {
            double dx = front_x - tracking_path.poses.at(i).pose.position.x;
            double dy = front_y - tracking_path.poses.at(i).pose.position.y;

            double dist = sqrt(dx*dx + dy*dy);
            if(dist<min_dist)
            {
                min_dist = dist;
                min_index = i;
            }
        }

        double map_x = tracking_path.poses.at(min_index).pose.position.x;
        double map_y = tracking_path.poses.at(min_index).pose.position.y;

        double dx = map_x - front_x;
        double dy = map_y - front_y;
        double vec_x = cos(vehicle_yaw + M_PI/2);
        double vec_y = sin(vehicle_yaw + M_PI/2);

        double cte = dx*vec_x + dy*vec_y;

        double yaw_term = cyaw - vehicle_yaw;
        double cte_term = atan2(k*cte, velocity());

        steering = (yaw_term + cte_term);
    }

    return steering;
}

void local_path::process()
{
    nav_msgs::Path temp_path;
    temp_path.header.stamp = ros::Time::now();
    temp_path.header.frame_id = "map";

    path_tracking();

    cmd_vel.longlCmdType = 2;
    cmd_vel.steering = stanley();
    cmd_vel.velocity = speed;

    cmd_pub.publish(cmd_vel);
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "local_path");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    local_path path;
    path.flag = false;
    path.speed = 8;

    ros::Rate loop_rate(path.rate);
    while(ros::ok())
    {
        path.process();
        loop_rate.sleep();
        ros::spinOnce();

        ROS_INFO("%f\t%f", path.pose.pose.position.x, path.pose.pose.position.y);
    }

    ros::spin();
    return 0;
}
