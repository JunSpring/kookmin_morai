#include "ros/ros.h"
#include "lidar_detection/lidarALL.h" 
#include "rubber_cone/rubber_cone_msg.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/Int32MultiArray.h"
#include "math.h"
#include <vector>
#include <algorithm>
#include <tuple>

#define _use_math_defines
#define MAX_DIST 1.4

class RubberCone
{
public:
    RubberCone(); 
    void Callback(const lidar_detection::lidarALL& lidar_msg);
    void publish_center(float center_x, float center_y);
    void dist_calc(float position_x, float position_y, bool update);

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub_center;
    ros::Publisher markerPub;
    
    rubber_cone::rubber_cone_msg cone_msg;

    float lidar0_distance;
    float lidar1_distance;
    float lidar2_distance;
    float lidar3_distance;
    float lidar4_distance;
    float lidar5_distance;
    bool center_update;

    std::vector<std::tuple<float, float, float>> dist_left;
    std::vector<std::tuple<float, float, float>> dist_right;
};