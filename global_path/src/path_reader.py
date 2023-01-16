#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class pathReader:
    def __init__(self,pkg_name):
        rospack=rospkg.RosPack()
        self.file_path=rospack.get_path(pkg_name)

    def read_txt(self,file_name):
        full_file_name = self.file_path+"/"+file_name
        openFile = open(full_file_name, 'r')
        out_path = Path()
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

if __name__ == '__main__':
    try:
        p_r=pathReader("global_path")
        global_pathes = [] #p_r.read_txt("webot_path")
        rospy.init_node('path_reader', anonymous=True)
        path_pubs = [] #rospy.Publisher('/path',Path, queue_size=1)

        path_name = rospy.get_param("~path_name")
        path_num = rospy.get_param("~path_num")
        
        for i in range(path_num):
            global_pathes.append(p_r.read_txt(path_name+str(i)+".txt"))
        for i in range(path_num):
            path_pubs.append(rospy.Publisher('/path'+str(i), Path, queue_size=1))

        rate=rospy.Rate(1)
        while not rospy.is_shutdown():
            text = ""
            for i, path_pub in enumerate(path_pubs):
                path_pub.publish(global_pathes[i])
                text += path_name+str(i)+".txt "

            if len(path_pubs) == 1:
                text += "is "
            else:
                text += "are "

            rospy.loginfo(text+"published!")
            rate.sleep()
    except rospy.ROSInterruptException:
        pass