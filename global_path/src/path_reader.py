#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from status.msg import status_msg

lane_num = 2

class pathReader :
    def __init__(self,pkg_name):
        rospack=rospkg.RosPack()
        self.file_path=rospack.get_path(pkg_name)

    def read_txt(self,file_name):
        full_file_name=self.file_path+"/"+file_name
        openFile = open(full_file_name, 'r')
        out_path=Path()
        out_path.header.frame_id='/map'
        # 파일 한줄 --> waypoint 한개
        line=openFile.readlines()
        for i in line :
            tmp=i.split()
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0])
            read_pose.pose.position.y=float(tmp[1])
            read_pose.pose.position.z=0
            read_pose.pose.orientation.x=0
            read_pose.pose.orientation.y=0
            read_pose.pose.orientation.z=0
            read_pose.pose.orientation.w=0
            out_path.poses.append(read_pose)

        openFile.close()
        return out_path

def callback(data):
    global lane_num
    lane_num = data.lane_num

if __name__ == '__main__':
    try:
        main_lane = pathReader("global_path")
        change_lane = pathReader("global_path")

        main_lane = main_lane.read_txt("webot_path0.txt")
        change_lane_2 = change_lane.read_txt("webot_path1.txt")
        change_lane_5 = change_lane.read_txt("webot_path2.txt")
        
        rospy.init_node('path_reader', anonymous=True)
        path_pub    = rospy.Publisher('/path',Path, queue_size=1)
        status_sub  = rospy.Subscriber("/status", status_msg, callback)

        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            if lane_num == 12:
                path_pub.publish(change_lane_2)
                rospy.loginfo("change lane(2) is published!")
            elif lane_num == 15:
                path_pub.publish(change_lane_5)
                rospy.loginfo("change lane(5) is published!")
            elif lane_num == 2:
                path_pub.publish(main_lane)
                rospy.loginfo("main lane is published")
            
            rate.sleep()
    except rospy.ROSInterruptException:
        pass