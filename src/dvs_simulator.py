#!/usr/bin/env python

import rospy
import rospkg
import rosbag
import cv2
import yaml
import OpenEXR
import Imath
import numpy as np
import os.path
import time

from std_msgs.msg import Float32
from dvs_msgs.msg import Event, EventArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def make_camera_msg(cam_data):
    camera_info_msg = CameraInfo()
    fx = cam_data['cam_fx']
    fy = cam_data['cam_fy']
    cx = cam_data['cam_cx']
    cy = cam_data['cam_cy']
    camera_info_msg.width = cam_data['cam_width']
    camera_info_msg.height = cam_data['cam_height']
    camera_info_msg.K = [fx, 0, cx,
                         0, fy, cy,
                         0, 0, 1]
                         
    camera_info_msg.D = [0, 0, 0, 0]
    
    camera_info_msg.P = [fx, 0, cx, 0,
                         0, fy, cy, 0,
                         0, 0, 1, 0]
    return camera_info_msg
    
    
    
def make_pose_msg(pose, timestamp):
    pose_msg = PoseStamped()
    pose_msg.header.stamp = timestamp
    pose_msg.header.frame_id = '/dvs_simulator'
    pose_msg.pose.position.x = pose[0]
    pose_msg.pose.position.y = pose[1]
    pose_msg.pose.position.z = pose[2]
    pose_msg.pose.orientation.x = pose[3]
    pose_msg.pose.orientation.y = pose[4]
    pose_msg.pose.orientation.z = pose[5]
    pose_msg.pose.orientation.w = pose[6]
    return pose_msg



def make_event(x, y, ts, pol):
    e = Event()
    e.x = x
    e.y = y
    e.ts = ts
    e.polarity = pol
    return e


class dvs_simulator:
    def __init__(self, init_sensor, C):
        self.sensor_ = init_sensor
        self.C_ = C

    def update_sensor(self, time, logI):        
        
        diff = logI - self.sensor_
        events_triggered = np.abs(diff) > self.C_
        self.sensor_[events_triggered] = logI[events_triggered]
        
        events = []
        
        # positive events
        Y, X = np.where(diff > self.C_)
        for i in range(len(X)):
            events.append(make_event(X[i], Y[i], time, True))

        # negative events
        Y, X = np.where(diff < -self.C_)
        for i in range(len(X)):
            events.append(make_event(X[i], Y[i], time, False))
            
        return events
        

# Linear color space to sRGB
# https://en.wikipedia.org/wiki/SRGB#The_forward_transformation_.28CIE_xyY_or_CIE_XYZ_to_sRGB.29
def lin2srgb(c):
    a = 0.055
    t = 0.0031308
    c[c <= t] = 12.92 * c[c <= t]
    c[c > t] = (1+a)*np.power(c[c > t], 1.0/2.4) - a
    return c
    
    
def safe_log(img):
    eps = 0.001
    return np.log(eps + img)


def extract_grayscale(img, map_to_srgb=False):
  dw = img.header()['dataWindow']

  size = (dw.max.x - dw.min.x + 1, dw.max.y - dw.min.y + 1)
  precision = Imath.PixelType(Imath.PixelType.FLOAT)
  R = img.channel('R', precision)
  G = img.channel('G', precision)
  B = img.channel('B', precision)
  
  r = np.fromstring(R, dtype = np.float32)
  g = np.fromstring(G, dtype = np.float32)
  b = np.fromstring(B, dtype = np.float32)
  
  r.shape = (size[1], size[0])
  g.shape = (size[1], size[0])
  b.shape = (size[1], size[0])
  
  if map_to_srgb:
      r = lin2srgb(r)
      g = lin2srgb(g)
      b = lin2srgb(b)
  
  rgb = cv2.merge([b, g, r])
  grayscale = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)

  return grayscale
  


def extract_depth(img):
  dw = img.header()['dataWindow']
  size = (dw.max.x - dw.min.x + 1, dw.max.y - dw.min.y + 1)
  precision = Imath.PixelType(Imath.PixelType.FLOAT)
  Z = img.channel('Z', precision)
  z = np.fromstring(Z, dtype = np.float32)
  z.shape = (size[1], size[0])
  return z
    
    

def parse_dataset(dataset_dir):
    
     # Parse camera calibration
    cam_file = open('%s/camera.yaml' % dataset_dir)
    cam_data = yaml.safe_load(cam_file)

    image_data = {}

    # Parse image paths       
    lines = [line.rstrip('\n') for line in open('%s/images.txt' % dataset_dir)]
    for line in lines:
        img_id, img_timestamp, img_path = line.split(' ')
        image_data[int(img_id)] = (rospy.Time(float(img_timestamp)), img_path)
    
        
    # Parse camera poses
    lines = [line.rstrip('\n') for line in open('%s/trajectory.txt' % dataset_dir)]
    for line in lines:
        splitted = line.split(' ')
        img_id = int(splitted[0])
        translation = [float(i) for i in splitted[1:4]]
        orientation = [float(i) for i in splitted[4:]]
        image_data[img_id] += (translation + orientation, )
        
    return image_data, cam_data



if __name__ == '__main__':

    rospy.init_node('dvs_simulator_node', anonymous=True)
    rospack = rospkg.RosPack()

    # Load simulator parameters
    dataset_name = rospy.get_param('dataset_name', 'dvs_full_mosaic')
    cp = rospy.get_param('contrast_p', 0.15)
    cm = rospy.get_param('contrast_m', -cp)
    sigp = rospy.get_param('sigma_p', 0.0)
    sigm = rospy.get_param('sigma_m', 0.0)
    
    event_streaming_rate = rospy.get_param('event_streaming_rate', 300)
    image_streaming_rate = rospy.get_param('image_streaming_rate', 24)
    write_to_bag = rospy.get_param('write_to_bag', False)
    
    rospy.loginfo('Dataset name: %s' % dataset_name)
    rospy.loginfo('Contrast threshold (+): %f' % cp)
    rospy.loginfo('Contrast threshold (-): %f' % cm)
    rospy.loginfo('Contrast threshold Std (+): %f' % sigp)
    rospy.loginfo('Contrast threshold Std (-): %f' % sigm)
    rospy.loginfo('Event streaming rate: %d packets / s' % event_streaming_rate)
    rospy.loginfo('Image streaming rate: %d images / s' % image_streaming_rate)
    
    delta_event = rospy.Duration(1.0 / event_streaming_rate)
    delta_image = rospy.Duration(1.0 / image_streaming_rate)
    
    # Parse dataset
    dataset_dir = os.path.join(rospack.get_path('rpg_datasets'), 'DVS', 'synthetic', 'full_datasets', dataset_name, 'data')
    image_data, cam_data = parse_dataset(dataset_dir)    
    camera_info_msg = make_camera_msg(cam_data)
    
    # Prepare publishers
    bridge = CvBridge()
    depthmap_pub = rospy.Publisher("/dvs/depthmap", Image, queue_size=0)
    image_pub = rospy.Publisher("/dvs/image_raw", Image, queue_size=0)
    pose_pub = rospy.Publisher("/dvs/pose", PoseStamped, queue_size=0)
    camera_info_pub = rospy.Publisher("/dvs/camera_info", CameraInfo, queue_size=0)
    event_pub = rospy.Publisher("/dvs/events", EventArray, queue_size=0)
    
    if write_to_bag:
        bag_dir = os.path.join(rospack.get_path('rpg_datasets'), 'DVS', 'synthetic', 'rosbags', dataset_name)
        if not os.path.exists(bag_dir):
            os.makedirs(bag_dir)
        bag = rosbag.Bag('%s/%s-%s.bag' % (bag_dir, dataset_name, time.strftime("%Y%m%d-%H%M%S")), 'w')
    
    # Initialize DVS
    first_frame = image_data.values()[0]
    exr_img = OpenEXR.InputFile('%s/%s' % (dataset_dir, first_frame[1]))
    init_sensor = safe_log(extract_grayscale(exr_img))
    init_time = first_frame[0]
    last_pub_img_timestamp = init_time
    last_pub_event_timestamp = init_time
    events = []
    dvs_sim = dvs_simulator(init_sensor, cp)
    
    if write_to_bag:
        bag.write(topic='/dvs/contrast_p', msg=Float32(cp), t=init_time)
        bag.write(topic='/dvs/contrast_m', msg=Float32(cm), t=init_time)
        bag.write(topic='/dvs/sigma2_p', msg=Float32(sigp), t=init_time)
        bag.write(topic='/dvs/sigma2_m', msg=Float32(sigm), t=init_time)
           
    if not write_to_bag:
        # Do not start publishing events if no one is listening
        rate = rospy.Rate(100)
        while event_pub.get_num_connections() == 0:
            rate.sleep()

    # Start simulation
    for frame in image_data.itervalues():
        
        if rospy.is_shutdown():
            break
        
        timestamp, img_path, pose = frame[:3]
        
        #rospy.loginfo('Processing frame at time: %f' % timestamp.to_sec())
            
        # publish pose
        if pose_pub.get_num_connections() > 0:
            pose_pub.publish(make_pose_msg(pose, timestamp))
        
        if write_to_bag:
            bag.write(topic='/dvs/pose', msg=make_pose_msg(pose, timestamp), t=timestamp)
            
        # publish camera_info
        if camera_info_pub.get_num_connections() > 0:
            camera_info_pub.publish(camera_info_msg)
            
        if write_to_bag:
            bag.write(topic='/dvs/camera_info', msg=camera_info_msg, t=timestamp)
        
        exr_img = OpenEXR.InputFile('%s/%s' % (dataset_dir, img_path))
        img = extract_grayscale(exr_img)
        
        
        if timestamp - last_pub_img_timestamp > delta_image or timestamp == init_time:
            # publish image_raw
            if write_to_bag or image_pub.get_num_connections > 0:
                img_msg = bridge.cv2_to_imgmsg(np.uint8(img * 255.0), 'mono8')
                img_msg.header.stamp = timestamp
                try:
                    image_pub.publish(img_msg)
                except CvBridgeError as e:
                    print(e)
                
                if write_to_bag:
                    bag.write(topic='/dvs/image_raw', msg=img_msg, t=timestamp)
                    
            # publish depth_map
            if write_to_bag or depthmap_pub.get_num_connections() > 0:
                z = extract_depth(exr_img)
                depth_msg = bridge.cv2_to_imgmsg(z, '32FC1')
                depth_msg.header.stamp = timestamp
                try:
                    depthmap_pub.publish(depth_msg)
                except CvBridgeError as e:
                        print(e)
 
                if write_to_bag:
                    bag.write(topic='/dvs/depthmap', msg=depth_msg, t=timestamp)
                
            last_pub_img_timestamp = timestamp
        
        # compute events for this frame
        logI = safe_log(img)
        events_cur = dvs_sim.update_sensor(timestamp, logI)
        events = events + events_cur
        
        # publish events
        if timestamp - last_pub_event_timestamp > delta_event:
            event_array = EventArray()
            event_array.header.stamp = timestamp
            event_array.width = cam_data['cam_width']
            event_array.height = cam_data['cam_height']
            event_array.events = events
            event_pub.publish(event_array)
            
            if write_to_bag:
                bag.write(topic='/dvs/events', msg=event_array, t=timestamp)
            
            events = []
            last_pub_event_timestamp = timestamp
            
    if write_to_bag:       
        bag.close()
        rospy.loginfo('Finished writing rosbag')
    
