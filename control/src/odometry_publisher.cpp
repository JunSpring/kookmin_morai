#include <odometry_publisher.h>

Odometry::Odometry()
{
  ros::NodeHandle nh;

  // Variable
  x = 0.0;
  y = 0.0;
  th = 0.0;
  
  // Publish
  odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);

  // Subscribe
  Ego_sub = nh.subscribe("/Ego_topic", 10, &Odometry::Egocallback, this);
  imu_sub = nh.subscribe("/imu", 10, &Odometry::imucallback, this);
  tf_sub = nh.subscribe("/tf", 10, &Odometry::tfcallback, this);
}

void Odometry::Egocallback(const morai_msgs::EgoVehicleStatus& msg)
{
  vx = msg.velocity.x;
  vy = msg.velocity.y;
}

void Odometry::imucallback(const sensor_msgs::Imu& msg)
{
  vth = msg.angular_velocity.z;
}

void Odometry::tfcallback(const tf2_msgs::TFMessage& msg)
{
  x = msg.transforms[0].transform.translation.x;
  y = msg.transforms[0].transform.translation.y;
}

void Odometry::process(ros::Time ct, ros::Time lt)
{
  double dt = (ct - lt).toSec();

  //compute odometry in a typical way given the velocities of the robot
  double delta_th = vth * dt;
  th += delta_th;

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  odom_quat = tf::createQuaternionMsgFromYaw(th);

  //next, we'll publish the odometry message over ROS
  odom.header.stamp = ct;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;

  //publish the message
  odom_pub.publish(odom);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle nh;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  Odometry od;

  ros::Rate r(od.rate);
  while(nh.ok()){
    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    od.process(current_time, last_time);

    last_time = current_time;
    r.sleep();
  }
}