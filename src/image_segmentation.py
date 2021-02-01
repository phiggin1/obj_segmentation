#!/usr/bin/env python

import rospy
import message_filters
import numpy as np
import image_geometry
import math
import cv2
from cv_bridge import CvBridge
from obj_segmentation.msg import SegmentedClustersArray
from obj_segmentation.msg import Object
from obj_segmentation.msg import ObjectArray
from obj_segmentation.srv import GetImages, GetImagesResponse
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image, CameraInfo
import sensor_msgs.point_cloud2 as pc2
from matplotlib import pyplot as plt 

class ImageSegment:
	def __init__(self):
		rospy.init_node('image_segment', anonymous=True)
		self.bridge = CvBridge()

		self.rgb_cam_info = rospy.wait_for_message("/camera/rgb/camera_info", CameraInfo, timeout=None)
		self.depth_cam_info = rospy.wait_for_message("/camera/depth/camera_info", CameraInfo, timeout=None)

		#self.cam_model = image_geometry.StereoCameraModel()
		#self.cam_model.fromCameraInfo(self.depth_cam_info, self.rgb_cam_info)

		self.cam_model = image_geometry.PinholeCameraModel()
		self.cam_model.fromCameraInfo(self.depth_cam_info)

		self.obj_pub = rospy.Publisher('/objects_images', ObjectArray, queue_size=10)

		self.rgb_image_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
		self.depth_image_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
		self.object_clusters_sub = message_filters.Subscriber('/object_clusters', SegmentedClustersArray)
		self.ts = message_filters.TimeSynchronizer([self.rgb_image_sub, self.depth_image_sub, self.object_clusters_sub], 10)
		self.ts.registerCallback(self.callback)

		#self.testpub = rospy.Publisher('/test', Image, queue_size=10)

		#self.obj_cluster_sub = rospy.Subscriber("object_clusters", SegmentedClustersArray, self.process_clusters)
		print("Service ready")
		self.service = rospy.Service('get_images', GetImages, self.process_clusters)

		rospy.spin()

	def callback(self, rgb_ros_image, depth_ros_image, object_clusters):
		self.object_clusters = object_clusters
		self.rgb = np.asarray(self.bridge.imgmsg_to_cv2(rgb_ros_image, desired_encoding="passthrough"))
		self.depth = np.asarray(self.bridge.imgmsg_to_cv2(depth_ros_image, desired_encoding="passthrough"))

	def process_clusters(self, req):
		print("recv serv req")
		object_array = ObjectArray()
		for i, pc in enumerate(self.object_clusters.clusters):
			#print("obj %d" % i)

			obj = Object()
			min_x = 1000.0
			min_y = 1000.0
			min_z = 1000.0
			max_x = -1000.0
			max_y = -1000.0
			max_z = -1000.0

			for p in pc2.read_points(pc):
				if p[0] > max_x:
					max_x = p[0]
				if p[0] < min_x:
					min_x = p[0]

				if p[1] > max_y:
					max_y = p[1]
				if p[1] < min_y:
					min_y = p[1]

				if p[2] > max_z:
					max_z = p[2]
				if p[2] < min_z:
					min_z = p[2]

			center = [(min_x + max_x)/2, (min_y + max_y)/2, (min_z + max_z)/2]
			w = max_x-min_x
			h = max_y-min_y
			d = max_z-min_z

			min_pix = self.cam_model.project3dToPixel( [ min_x, min_y, min_z ] )
			max_pix = self.cam_model.project3dToPixel( [ max_x, max_y, max_z ] )

			u_min = max(int(math.floor(min_pix[0]))-25, 0)
			v_min = max(int(math.floor(min_pix[1]))-25, 0)
			
			u_max = min(int(math.ceil(max_pix[0]))+25, self.rgb.shape[1])
			v_max = min(int(math.ceil(max_pix[1]))+25, self.rgb.shape[1])

			rgb_cropped = self.rgb[v_min:v_max, u_min:u_max].copy()	
			depth_cropped = self.depth[v_min:v_max, u_min:u_max].copy()
			
			obj.loc.header = pc.header
			obj.loc.point.x = center[0]
			obj.loc.point.y = center[1]
			obj.loc.point.z = center[2]

			obj.depth = self.bridge.cv2_to_imgmsg(depth_cropped, encoding="32FC1")
			obj.rgb =  self.bridge.cv2_to_imgmsg(rgb_cropped, encoding="bgr8")

			obj.depth.header = pc.header
			obj.rgb.header = pc.header

			'''
			if i==0: 
				self.testpub.publish(obj.rgb)
			'''
			object_array.objects.append(obj)

		#self.obj_pub.publish(object_array)
		return GetImagesResponse(object_array)

	
if __name__ == '__main__':
    segmenter = ImageSegment()

