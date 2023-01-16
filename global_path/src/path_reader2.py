#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

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

if __name__ == '__main__':
    try:
        inside =pathReader("global_path")
        outside = pathReader("global_path")
        inside_path = inside.read_txt("inside.txt")
        outside_path = outside.read_txt("outside.txt")
        rospy.init_node('path_reader', anonymous=True)
        path_pub = rospy.Publisher('/path',Path, queue_size=1)

        is_outside = True
        change_time = rospy.get_time()

        rate=rospy.Rate(1)
        while not rospy.is_shutdown():
            if rospy.get_time() - change_time > 2:
                is_outside = not is_outside
                change_time = rospy.get_time()
                # print(is_outside)

            if is_outside:
                path_pub.publish(outside_path)
            else:
                path_pub.publish(inside_path)
            
            rate.sleep()
    except rospy.ROSInterruptException:
        pass