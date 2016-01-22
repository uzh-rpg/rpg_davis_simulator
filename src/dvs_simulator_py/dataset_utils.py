#!/usr/bin/env python

import yaml
import cv2
import Imath
import numpy as np
from vikit_py import transformations as transformations
from vikit_py import pinhole_camera  


""" Parse a dataset folder """
def parse_dataset(dataset_dir):
    
     # Parse camera calibration
    cam_file = open('%s/camera.yaml' % dataset_dir)
    cam_data = yaml.safe_load(cam_file)

    image_data = {}

    # Parse image paths       
    lines = [line.rstrip('\n') for line in open('%s/images.txt' % dataset_dir)]
    for line in lines:
        img_id, img_timestamp, img_path = line.split(' ')
        image_data[int(img_id)] = (float(img_timestamp), img_path)
    
     
    # Parse camera trajectory
    lines = [line.rstrip('\n') for line in open('%s/trajectory.txt' % dataset_dir)]
    for line in lines:
        splitted = line.split(' ')
        img_id = int(splitted[0])
        translation = [float(i) for i in splitted[1:4]]
        orientation = [float(i) for i in splitted[4:]]
        image_data[img_id] += (translation + orientation, )
        
    t = [frame[0] for frame in image_data.itervalues()]
    positions = [frame[2][:3] for frame in image_data.itervalues()]
    orientations = [frame[2][-4:] for frame in image_data.itervalues()]
    img_paths = [frame[1] for frame in image_data.itervalues()]
    
    width = cam_data['cam_width']
    height = cam_data['cam_height']
    fx = cam_data['cam_fx']
    fy = cam_data['cam_fy']
    cx = cam_data['cam_cx']
    cy = cam_data['cam_cy']
    
    cam = pinhole_camera.PinholeCamera(width, height, fx, fy, cx, cy)
        
    return t, img_paths, positions, orientations, cam
   
   
   
""" Linear color space to sRGB
    https://en.wikipedia.org/wiki/SRGB#The_forward_transformation_.28CIE_xyY_or_CIE_XYZ_to_sRGB.29 """
def lin2srgb(c):
    a = 0.055
    t = 0.0031308
    c[c <= t] = 12.92 * c[c <= t]
    c[c > t] = (1+a)*np.power(c[c > t], 1.0/2.4) - a
    return c


   

def extract_grayscale(img):
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


    
    
""" Compute linear and angular velocities along a trajectory """
def linear_angular_velocity(t, positions, orientations, stride):
    w_body = np.zeros((len(t)-stride, 3))
    v_world = np.zeros((len(t)-stride, 3))
    v_body = np.zeros((len(t)-stride, 3))
    for i in range(0, len(t)-stride, stride):
        dt = t[i+stride] - t[i]
        R_wb_t = transformations.matrix_from_quaternion(orientations[i])[:3,:3]
        R_wb_t_dt = transformations.matrix_from_quaternion(orientations[i+stride])[:3,:3]
        
        v_world[i,:] = 1.0 / dt * (np.array(positions[i+stride])-np.array(positions[i]))
        v_body[i,:] = R_wb_t.transpose().dot(v_world[i,:])
        
        # w(t) = 1/dt * log(R_wb(t)^T * R_wb(t+dt))
        w_body[i,:] = 1.0 / dt * transformations.logmap_so3(R_wb_t.transpose().dot(R_wb_t_dt))
        
    return v_world, v_body, w_body
