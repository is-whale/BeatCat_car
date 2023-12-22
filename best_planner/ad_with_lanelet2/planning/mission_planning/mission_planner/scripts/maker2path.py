#!/usr/bin/env python3
import sys,os
import time,datetime
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Route2Path:
    def __init__(self):
        self.path = Path()
        self.pathpub = rospy.Publisher('/global_path', Path, queue_size=10)
        self.rate = rospy.Rate(10)
        # print("12")
        rospy.Subscriber('/planning/mission_planning/route_marker', MarkerArray, self.callback)

    def callback(self, msg):
        # print("123")
        # print(self.postions )
        for marker in msg.markers:
            print(marker.ns)
            if marker.ns == 'right_lane_bound':
            # if marker.ns == 'route_lanelets':
                self.postions = marker.points
                for i in range(0,len(self.postions)):
                    self.pose = PoseStamped()
                    self.pose.pose.position.x = self.postions[i].x
                    self.pose.pose.position.y = self.postions[i].y
                    self.pose.pose.position.z = self.postions[i].z
                    self.pose.pose.orientation.x  = 0.0
                    self.pose.pose.orientation.y  = 0.0
                    self.pose.pose.orientation.z  = 0.0
                    self.pose.pose.orientation.w = 1.0
                    # print(self.pose)
                for i in range(0,len(self.postions)):

                    self.path.poses.append(self.pose)
        self.publish_path()

    def publish_path(self):
        # print(self.path.poses)
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = "map"
        while not rospy.is_shutdown():
            self.pathpub.publish(self.path)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('route2Path_node', anonymous=True)
        route2path_node = Route2Path()
        rospy.spin()
    except Exception as e:
        print(e)
        raise e
