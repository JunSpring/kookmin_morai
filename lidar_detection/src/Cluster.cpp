#include "Cluster.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/Int32MultiArray.h"

CloudCluster::CloudCluster()
{
    sub = nh.subscribe("/pcl2", 1, &CloudCluster::Callback, this); // PCL2 Subscribe
    pub_lidarALL = nh.advertise<lidar_detection::lidarALL>("/lidar_info", 1); // Lidar Info Publish
    markerPub = nh.advertise<visualization_msgs::MarkerArray>("viz", 1); // Marker Publish

    // 장애물 위치를 따로 publish 하는 경우
    // pub_lidar0= nh.advertise<lidar_detection::lidar0>("/lidar_info0", 1); 
    // pub_lidar1= nh.advertise<lidar_detection::lidar1>("/lidar_info1", 1);
    // pub_lidar2= nh.advertise<lidar_detection::lidar2>("/lidar_info2", 1);
}

void CloudCluster::publish_msg(int num, float position_x, float position_y) 
{
    switch(num)
    {
        case 0:
        {
            // lidar0.num = num;
            // lidar0.position_x = position_x;
            // lidar0.position_y = position_y;
            // pub_lidar0.publish(lidar0);

            lidarALL.position_x0 = position_x;
            lidarALL.position_y0 = position_y;
            lidar0_update = true;
            break;
        }

        case 1:
        {
            // lidar1.num = num;
            // lidar1.position_x = position_x;
            // lidar1.position_y = position_y;
            // pub_lidar1.publish(lidar1);

            lidarALL.position_x1 = position_x;
            lidarALL.position_y1 = position_y;
            lidar1_update = true;
            break;
        }

        case 2:
        {
            // lidar2.num = num;
            // lidar2.position_x = position_x;
            // lidar2.position_y = position_y;
            // pub_lidar2.publish(lidar2);

            lidarALL.position_x2 = position_x;
            lidarALL.position_y2 = position_y;
            lidar2_update = true;
            break;
        }

        case 3:
        {
            lidarALL.position_x3 = position_x;
            lidarALL.position_y3 = position_y;
            lidar3_update = true;
            break;
        }

        case 4:
        {
            lidarALL.position_x4 = position_x;
            lidarALL.position_y4 = position_y;
            lidar4_update = true;
            break;
        }

        case 5:
        {
            lidarALL.position_x5 = position_x;
            lidarALL.position_y5 = position_y;
            lidar5_update = true;
            break;
        }

        default:
            break;
    }
}
    

void CloudCluster::Callback(const sensor_msgs::PointCloud2ConstPtr& pcl2_msg)
{
    lidar0_update = false;
    lidar1_update = false;
    lidar2_update = false;
    lidar3_update = false;
    lidar4_update = false;
    lidar5_update = false;
    
    int n = 6;
    std::vector<geometry_msgs::Point> KFpredictions;
    visualization_msgs::MarkerArray clusterMarkers;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>()); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    std::vector<pcl::PointIndices> cluster_indices;

    pcl::fromROSMsg(*pcl2_msg, *input_cloud); // ROS Message cloud2를 cloud로 변환  
    tree->setInputCloud(input_cloud); 

    // Euclidean Cluster Extraction
    ec.setClusterTolerance(0.08);    // Point 사이 최소 허용 거리
    ec.setMinClusterSize(4);    // Min Cluster Size
    ec.setMaxClusterSize(30);   // Max Cluster Size
    ec.setSearchMethod(tree);   // Search Method - tree 
    ec.setInputCloud(input_cloud);    // Clustering result
    ec.extract(cluster_indices);
    
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) 
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        float x = 0.0;
        float y = 0.0;
        int numPts = 0;

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) 
        {
            cloud_cluster->points.push_back(input_cloud->points[*pit]);
            x += input_cloud->points[*pit].x;
            y += input_cloud->points[*pit].y;
            numPts++;
        }

        geometry_msgs::Point pt;
        pt.x = x / numPts;
        pt.y = y / numPts;
        pt.z = 0.0;

        if (pt.x != 0 && pt.y != 0)
        {
            KFpredictions.push_back(pt);
        }
        
    }

    if (KFpredictions.size()!=0)
    {
        visualization_msgs::Marker m;
        for (int i = 0; i < n; i++) 
        {
            m.id = i;
            m.type = visualization_msgs::Marker::CYLINDER;
            m.header.frame_id = "map";
            m.scale.x = 0.3;
            m.scale.y = 0.3;
            m.scale.z = 0.3;
            m.color.a = 1.0;
            m.color.r = 1;
            m.color.g = 0;
            m.color.b = 0;
            // m.color.r = i == 0 ? 1 : 0;
            // m.color.g = i == 1 ? 1 : 0;
            // m.color.b = i == 2 ? 1 : 0;

            geometry_msgs::Point clusterC(KFpredictions[i]);

            if (abs(clusterC.x) < 2.0 && abs(clusterC.y) < 2.0)
            {
                m.pose.position.x = clusterC.x;
                m.pose.position.y = clusterC.y;
                m.pose.position.z = clusterC.z;
            }
        
            if (abs(m.pose.position.x) > 0.01 && abs(m.pose.position.y) > 0.01 && abs(m.pose.position.x) < 1.75 && abs(m.pose.position.y) < 1.75)
            {
                m.action = visualization_msgs::Marker::ADD;
                clusterMarkers.markers.push_back(m);     
                publish_msg(i, m.pose.position.x, m.pose.position.y);
                // ROS_INFO("NO.%d x: %f y: %f", i, m.pose.position.x, m.pose.position.y);
            }

            else
            {
                m.action = visualization_msgs::Marker::DELETE;
                clusterMarkers.markers.push_back(m);  
                // ROS_INFO("x: %f y: %f", m.pose.position.x, m.pose.position.y);
            }
        }
    }

    lidarALL.lidar0_update = lidar0_update;
    lidarALL.lidar1_update = lidar1_update;
    lidarALL.lidar2_update = lidar2_update;
    lidarALL.lidar3_update = lidar3_update;
    lidarALL.lidar4_update = lidar4_update;
    lidarALL.lidar5_update = lidar5_update;

    pub_lidarALL.publish(lidarALL);
    markerPub.publish(clusterMarkers);

    //ros::Rate loop_rate(5);
    //loop_rate.sleep(); 
}
