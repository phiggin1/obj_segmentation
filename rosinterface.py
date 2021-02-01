import base64
import roslibpy
import cv2
import numpy as np
from matplotlib import pyplot as plt 
import logging
import time

class ROSInterface:
    def __init__(self):
        self.client = roslibpy.Ros(host='localhost', port=9090)
        self.client.run()

        self.service = roslibpy.Service(self.client, 'get_images', 'obj_segmentation/ObjectArray')

        self.publisher = roslibpy.Topic(self.client, '/indicate', 'geometry_msgs/PointStamped')
        self.publisher.advertise()


    def get_images(self):
        request = roslibpy.ServiceRequest()
        #print('Calling service...')
        result = self.service.call(request)
        #print('Received %d objects' % len(result['resp']['objects']))
        i = 0
        images = []
        for obj in result['resp']['objects']:
            #print(i)
            point = obj['loc']
            rgb_w = obj['rgb']['width']
            rgb_h = obj['rgb']['height']
            rgb_encoding = obj['rgb']['encoding']
            rgb_step = obj['rgb']['step']
            rgb_data = obj['rgb']['data'].encode('ascii')
            rgb_bytes = base64.b64decode(rgb_data)
            rgb_im   = np.ndarray(shape=(rgb_h, rgb_w, 3),dtype=np.uint8, buffer=rgb_bytes)
            rgb = cv2.cvtColor(rgb_im, cv2.COLOR_BGR2RGB)

            depth_w = obj['depth']['width']
            depth_h = obj['depth']['height']
            depth_encoding = obj['depth']['encoding']
            depth_step = obj['depth']['step']
            depth_data = obj['depth']['data'].encode('ascii')
            depth_bytes = base64.b64decode(depth_data)
            depth = np.ndarray(shape=(depth_h, depth_w), dtype=np.float32, buffer=depth_bytes)
            images.append( (point, rgb, depth) )
            i += 1

        return images
    
    def idicate(self, obj):
        print(obj)
        self.publisher.publish(obj)

a = ROSInterface()

img = a.get_images()

print(len(img))

for im in img:
    f, axarr = plt.subplots(2,1) 
    axarr[0].imshow(im[1])
    axarr[1].imshow(im[2])
    plt.show()
    a.idicate(im[0])
    time.sleep(2.0)



