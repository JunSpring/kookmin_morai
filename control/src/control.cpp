#include "control.h"

local_path::local_path()
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param("VL",VL,0.3);                 // 차량 길이 (모라이 공식 WeCar 길이)
    pnh.param("max_lfd", max_lfd, 30.0);    // 최대 Looking foward
    pnh.param("min_lfd", min_lfd, 1.0);     // 최소 Looking foward
    pnh.param("num",num,30);                // tracking path 길이

    // Publish
    speed_pub = nh.advertise<std_msgs::Float64>("/commands/motor/speed", 10);
    position_pub = nh.advertise<std_msgs::Float64>("/commands/servo/position", 10);
    path_pub = nh.advertise<nav_msgs::Path>("tracking_path",10);
    marker_lfd = nh.advertise<visualization_msgs::MarkerArray>("/Look_Forward_Distance",10);
    local_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/local_goal", 10);

    // Subscibe
    path_sub = nh.subscribe("/path",10,&local_path::pathcallback,this);
    tf_sub = nh.subscribe("/tf", 10, &local_path::tfcallback, this);
    imu_sub = nh.subscribe("/imu", 10, &local_path::imucallback, this);
    status_sub = nh.subscribe("/status", 10, &local_path::statuscallback, this);
}

void local_path::pathcallback(const nav_msgs::Path& msg)
{
    path = msg;
    flag = true;
}

void local_path::tfcallback(const tf2_msgs::TFMessage& msg)
{
    tfmsg = msg;

    // 모라이에서 subscribe한 tf를 현재 pose로 변환
    pose.pose.position.x = msg.transforms[0].transform.translation.x;
    pose.pose.position.y = msg.transforms[0].transform.translation.y;
    pose.pose.position.z = msg.transforms[0].transform.translation.z;
    pose.pose.orientation.x = msg.transforms[0].transform.rotation.x;
    pose.pose.orientation.y = msg.transforms[0].transform.rotation.y;
    pose.pose.orientation.z = msg.transforms[0].transform.rotation.z;
    pose.pose.orientation.w = msg.transforms[0].transform.rotation.w;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";

    tf::Quaternion car_orientation(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    tf::Matrix3x3 matrix(car_orientation);
    matrix.getRPY(vehicle_roll, vehicle_pitch, vehicle_yaw);
}

void local_path::statuscallback(const status::status_msg& msg)
{
    status      = msg.status;
    lane_num    = msg.lane_num;
    mission3_go = msg.mission3_go;
    mission5_go = msg.mission5_go;
}

// 임시 imu 콜백
// local_goal에 대한 orientation이 고정되는 현상이 있는데 이를 통해 임시로 변경
// 후에 orientation에 대한 코드 작성 요망
void local_path::imucallback(const sensor_msgs::Imu& msg)
{
    bx = msg.orientation.x;
    by = msg.orientation.y;
    bz = msg.orientation.z;
    bw = msg.orientation.w;
}

// 모든 path에서 앞으로 나아갈 num만큼의 길이의 path를 /tracking_path로 publish
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
                    if(i == temp_num + num - 1)
                    {
                        tracking_path.poses.at(15).pose.orientation.x = bx;
                        tracking_path.poses.at(15).pose.orientation.y = by;
                        tracking_path.poses.at(15).pose.orientation.z = bz;
                        tracking_path.poses.at(15).pose.orientation.w = bw;
                        local_goal_pub.publish(tracking_path.poses.at(15));
                    }
            }
        }
        else
        {
            for(int i = temp_num; i< path.poses.size(); i++)
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
            for(int i = 0; i < num - (path.poses.size() - temp_num); i++)
            {
                temp_pose.pose.position.x = path.poses.at(i).pose.position.x;
                temp_pose.pose.position.y = path.poses.at(i).pose.position.y;
                temp_pose.pose.position.z = 0;
                tracking_path.poses.push_back(temp_pose);

                if(i == num - (path.poses.size() - temp_num) - 1)
                {
                    temp_pose.pose.orientation.x = 0.0;
                    temp_pose.pose.orientation.y = 0.0;
                    temp_pose.pose.orientation.z = 0.0;
                    temp_pose.pose.orientation.w = 1.0;
                    local_goal_pub.publish(temp_pose);
                }
            }
            // flag = false;
        }
        path_pub.publish(tracking_path);
    }
    else
    {
        ROS_INFO("DONE");
    }
}

double local_path::pure_pursuit()
{
    geometry_msgs::Pose index;
    is_look_foward_point = false;
    double min_dist = 10;
    double min_index = 0;
    double steering = 0;

    if(tracking_path.poses.size() > 1)
    {
        for(int i = 0; i<path.poses.size(); i++)
        {
            double dx = pose.pose.position.x - path.poses.at(i).pose.position.x;
            double dy = pose.pose.position.y - path.poses.at(i).pose.position.y;

            double dist = sqrt(dx*dx + dy*dy);
            if(dist<min_dist)
            {
                min_dist = dist;
                min_index = i;
            }
        }

        double x = pose.pose.position.x - path.poses.at(min_index).pose.position.x;
        double y = pose.pose.position.y - path.poses.at(min_index).pose.position.y;

        double distance = sqrt(pow(x,2) + pow(y,2));
        double dis = 0;

        double lfd = 1;
        double max_lfd = this->max_lfd;
        double min_lfd = this->min_lfd;
        double rotated_x = 0;
        double rotated_y = 0;

        lfd = speed.data / offset / 8.1;
        
        if(lfd < min_lfd)
        {
            lfd = min_lfd;
        }
        else if(lfd > max_lfd)
        {
            lfd = max_lfd;
        }

        for(int i = 0; i<tracking_path.poses.size(); i++)
        {
            double dx = tracking_path.poses.at(i).pose.position.x - pose.pose.position.x;
            double dy = tracking_path.poses.at(i).pose.position.y - pose.pose.position.y;

            rotated_x = cos(vehicle_yaw)*dx + sin(vehicle_yaw)*dy;
            rotated_y = sin(vehicle_yaw)*dx - cos(vehicle_yaw)*dy;

            if(rotated_x > 0)
            {
                dis = sqrt(pow(rotated_x,2) + pow(rotated_y,2));
                // ROS_INFO("%f\t%f", dis, lfd);
                if(dis>=lfd)
                {
                    index.position.x = tracking_path.poses.at(i).pose.position.x;
                    index.position.y = tracking_path.poses.at(i).pose.position.y;
                    is_look_foward_point = true;
                    break;
                }
            }
        }

        //ROS_INFO_STREAM( "Look_Forward_Distance = " << lfd);
        //lfd_marker
        visualization_msgs::MarkerArray node_arr;
        visualization_msgs::Marker node1;
        node1.header.frame_id = "map"; // map frame 기준
        node1.header.stamp = ros::Time::now();
        node1.type = visualization_msgs::Marker::SPHERE;
        node1.id = 0;
        node1.action = visualization_msgs::Marker::ADD;
        node1.pose.orientation.w = 1.0;
        node1.pose.position.x = index.position.x; //노드의 x 좌표
        node1.pose.position.y = index.position.y; //노드의 y 좌표 // Points are green
        node1.color.g = 0.5;
        node1.color.a = 1.0;
        node1.scale.x = 1;
        node1.scale.y = 1;
        node_arr.markers.push_back(node1);
        marker_lfd.publish(node_arr);
        //end

        double theta = atan2(rotated_y,rotated_x);
        if(is_look_foward_point == true)
        {
            double eta = atan2((2*VL*sin(theta)),lfd);
            steering = -eta;
        }
        else
        {
            ROS_INFO("no found forwad point");
        }
    }

    ROS_INFO("%f", steering * 180 / 3.14);
    return steering * 180 / 3.14;
}

// 모라이에서 steering angle이 -30° ~ 30°이므로 모든 각도를 이 사이 각도로 설정
double local_path::nomalize_angle(double angle)
{
    if (angle > 19.5)
    {
        angle = 19.5;
    }
    else if (angle < -19.5)
    {
        angle = -19.5;
    }

    return angle;
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

    return steering * 180 / 3.14;
}

void local_path::mission()
{
    switch(status)
    {
    case NM:
        go(8.1);
        break;
    case TL:
        if(mission3_go)
            go(8.1);
        else
            stop();
        break;
    case MO:
        if(mission5_go)
            go_slow(4.0);
        else
            stop();
        break;
    case WD:
        go_slow(2.0);
        break;
    default:
        go(8.1);
        break;
    }
}

void local_path::process()
{
    nav_msgs::Path temp_path;
    temp_path.header.stamp = ros::Time::now();
    temp_path.header.frame_id = "map";

    path_tracking();

    steering_angle = nomalize_angle(pure_pursuit());
    mission();
    
    speed_pub.publish(speed);
    position_pub.publish(position);
}

void local_path::go(double speed_input)
{
    speed.data = speed_input * offset / (abs(steering_angle) / 19.5 * 2.5 + 1);
    position.data = (steer_offset - steering_angle)/(steer_offset * 2);  //-1 * angle / 19.5 * 0.6353 + 0.5304;
}

void local_path::go_slow(double speed_input)
{
    speed.data = speed_input * offset;
    position.data = (steer_offset - steering_angle)/(steer_offset * 2);  //-1 * angle / 19.5 * 0.6353 + 0.5304;
}

void local_path::stop()
{
    speed.data = 0;
    position.data = (steer_offset - steering_angle)/(steer_offset * 2);  //-1 * angle / 19.5 * 0.6353 + 0.5304;
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "local_path");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    local_path path;
    path.flag = false;
    path.speed.data = 3 * 1170;

    ros::Rate loop_rate(path.rate);
    while(ros::ok())
    {
        path.process();
        loop_rate.sleep();
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}
