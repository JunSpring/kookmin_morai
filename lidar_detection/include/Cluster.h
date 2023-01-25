#include "Scan2PCL.h"
#include "lidar_detection/lidarALL.h" 
#include "lidar_detection/lidar0.h" 
#include "lidar_detection/lidar1.h" 
#include "lidar_detection/lidar2.h" 
#include "pcl/search/kdtree.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/Int32MultiArray.h"


class CloudCluster
{
public:
    CloudCluster(); 
    void Callback(const sensor_msgs::PointCloud2ConstPtr& pcl2_msg);
    void publish_msg(int num, float position_x, float position_y);

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub_lidarALL;
    ros::Publisher markerPub;
    // ros::Publisher pub_lidar0;
    // ros::Publisher pub_lidar1;
    // ros::Publisher pub_lidar2;
    
    lidar_detection::lidarALL lidarALL;
    // lidar_detection::lidar0 lidar0; 
    // lidar_detection::lidar1 lidar1;
    // lidar_detection::lidar2 lidar2;

    bool lidar0_update;
    bool lidar1_update;
    bool lidar2_update;
    bool lidar3_update;
    bool lidar4_update;
    bool lidar5_update;
};