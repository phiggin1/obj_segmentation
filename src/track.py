#!/usr/bin/env python

import rospy
import numpy as np
import struct
import ctypes
import random
import time
import math

from obj_segmentation.msg import SegmentedClustersArray

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray



THRESHOLD  = 0.005

def distance_point_to_plane(p, model):
    return ( model[0]*p[0] + model[1]*p[1] + model[2]*p[2] + model[3] )

def marker2str(marker):
    return str(marker.text +" " + format(marker.pose.position.x, '.3f')) + " " + str(format(marker.pose.position.y, '.3f')) + " " + str(format(marker.pose.position.z, '.3f'))

def get_stamped_point(x,y,z, frame_id):
    p = PointStamped()
    p.header.frame_id = frame_id
    p.header.stamp = rospy.Time.now()
    p.point.x = x
    p.point.y = y
    p.point.z = z

    return p

def get_marker(i, label, frame_id, x, y, z):
    marker = Marker()

    marker.id = i
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()

    marker.type = marker.TEXT_VIEW_FACING
    #marker.type = marker.SPHERE
    marker.text = label

    marker.action = marker.ADD

    marker.scale.z = 0.025

    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0        
    marker.color.a = 1.0

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.pose.position.x = float(x)
    marker.pose.position.y = float(y)
    marker.pose.position.z = float(z)

    return marker

def dist(a, b):
    ax = a.pose.position.x
    ay = a.pose.position.y
    az = a.pose.position.z
    
    bx = b.pose.position.x
    by = b.pose.position.y
    bz = b.pose.position.z
    
    return math.sqrt( (ax-bx)**2 + (ay-by)**2 + (az-bz)**2 )

def find_closest(old_obj, new_objects):
    min_dist = 999.9
    min_indx = 0
    for i, new_obj in enumerate(new_objects):
        d = dist(old_obj, new_obj)
        if  d < min_dist:
            min_dist = d
            min_indx = i

    return min_indx, min_dist

class Segmentation:
    def get_clusters(self, object_clusters):
        marker_array = MarkerArray()      

        #for all j clusters
        print("# clusters ", len(object_clusters.clusters))
        for i, pc in enumerate(object_clusters.clusters):
            num_points = pc.width * pc.height
            print("Cluster", i ,": # points ", num_points)

            sum_x = 0
            sum_y = 0
            sum_z = 0
            #for all points in the cluster i
            for p in pc2.read_points(pc):
                sum_x = sum_x + p[0]
                sum_y = sum_y + p[1]
                sum_z = sum_z + p[2]

            x = sum_x/num_points
            y = sum_y/num_points
            z = sum_z/num_points

            label = "obj_" + str(i)
            marker_array.markers.append(get_marker(i, label, object_clusters.header.frame_id, x, y, z))

        self.track(marker_array)
    
    def track(self, marker_array):
        if self.objects == None:
            self.objects = []
            for marker in marker_array.markers:
                self.objects.append(marker)
            print(len(self.objects), " initial objects")
        else:
            old_objects = self.objects
            new_objects = marker_array.markers
            
            old_indxs = list(range(len(old_objects)))
            new_indxs = list(range(len(new_objects)))

            for i, new in enumerate(new_objects):
                closest_indx, min_dist = find_closest(new, old_objects)
                #print(i, marker2str(new), closest_indx, marker2str(old_objects[closest_indx]), format(min_dist, '.3f'))
                if min_dist < THRESHOLD:
                    if i in new_indxs: 
                        new_indxs.remove(i)
                    if closest_indx in old_indxs: 
                        old_indxs.remove(closest_indx)

            if len(old_indxs) > len(new_indxs):
                print("things removed")
                #TODO Handle in future

            elif len(old_indxs) < len(new_indxs):
                print("things added")
                #TODO Handle in future
            
            if len(old_indxs) == len(new_indxs) and len(old_indxs) > 0:
                print("things have moved")
                print(old_indxs, new_indxs)
                for old_indx in old_indxs:
                    new = [new_objects[i] for i in new_indxs]
                    closest_indx, min_dist = find_closest(self.objects[old_indx], new)
    
                    self.objects[old_indx].pose.position.x = new[closest_indx].pose.position.x
                    self.objects[old_indx].pose.position.y = new[closest_indx].pose.position.y
                    self.objects[old_indx].pose.position.z = new[closest_indx].pose.position.z

            for obj in self.objects:
                obj.header.seq = obj.header.seq + 1
                obj.header.stamp = rospy.Time.now()
                print( marker2str(obj) )

            self.tracked_objects.publish(self.objects)
    
    def __init__(self):
        rospy.init_node('ransac_filter', anonymous=True)
        self.objects = None
        self.objects_sub = rospy.Subscriber('/object_clusters', SegmentedClustersArray, self.get_clusters)
        #self.object_pub = rospy.Publisher('/object', PointCloud2, queue_size=10)
        self.tracked_objects = rospy.Publisher('/tracked_objects', MarkerArray, queue_size=10)
        rospy.spin()

if __name__ == '__main__':
    segment = Segmentation()