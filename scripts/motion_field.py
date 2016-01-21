#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 19 17:58:47 2016

Load some synthetic dataset and display the motion field

@author: Henri Rebecq
"""

import rospy
import rospkg
import cv2
import yaml
import OpenEXR
import Imath
import numpy as np
import os.path
from matplotlib import pyplot as plt
from matplotlib import cm as cm
from vikit_py import transformations as transformations
from vikit_py import pinhole_camera as cameras


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


""" Compute motion vector numerically (finite-differencing) """
def motion_vector_numeric(cam, u, v, Z, frame_id, t, positions, quaternions):
    
    if frame_id == len(positions):
        raise IndexError('Cannot compute linear/angular velocity for last pose')
    
    dt = t[frame_id+1] - t[frame_id]
    T_w_t_dt = transformations.matrix_from_quaternion(quaternions[frame_id+1])
    T_w_t_dt[:3,3] = positions[frame_id+1]
    
    T_w_t = transformations.matrix_from_quaternion(quaternions[frame_id])
    T_w_t[:3,3] = positions[frame_id]
    
    T_new_old = np.linalg.inv(T_w_t_dt).dot(T_w_t)
    
    u_t = np.array([u, v])
    f = cam.back_project(u_t)
    f /= np.linalg.norm(f)
    
    depth_along_ray = Z / f[2]
    p_t = depth_along_ray * f
    
    p_t_dt = T_new_old[:3,:3].dot(p_t) + T_new_old[:3,3]
    
    u_t_dt, res = cam.project(p_t_dt)
    
    if not res:
        raise RuntimeWarning('Projection in next frame does not fall on image plane')
        
    return (u_t_dt - u_t) / dt
    
    
# Main routine    

rospack = rospkg.RosPack()
plt.close('all')

# User-defined parameters
dataset_name = 'one_textured_plane_helicoidal'

dataset_dir = os.path.join(rospack.get_path('rpg_datasets'), 'DVS', 'synthetic', 'full_datasets', dataset_name, 'data')

image_data, cam_data = parse_dataset(dataset_dir)

width = cam_data['cam_width']
height = cam_data['cam_height']
fx = cam_data['cam_fx']
fy = cam_data['cam_fy']
cx = cam_data['cam_cx']
cy = cam_data['cam_cy']

K = np.array([[fx, 0.0, cx],[0.0, fy, cy],[0.0, 0.0, 1.0]])
Kinv = np.linalg.inv(K)

t = [frame[0].to_sec() for frame in image_data.itervalues()]
positions = [frame[2][:3] for frame in image_data.itervalues()]
quaternions = [frame[2][-4:] for frame in image_data.itervalues()]
 
stride = 1 
w_body = np.zeros((len(t)-stride, 3))
#w_world = np.zeros((len(t)-stride, 3))
v_world = np.zeros((len(t)-stride, 3))
v_body = np.zeros((len(t)-stride, 3))
for i in range(0, len(t)-stride, stride):
    t_new = t[i+stride]
    dt = t[i+stride] - t[i]
    R_wb_t = transformations.matrix_from_quaternion(quaternions[i])[:3,:3]
    R_wb_t_dt = transformations.matrix_from_quaternion(quaternions[i+stride])[:3,:3]
    w_body[i,:] = 1.0 / dt * transformations.logmap_so3(R_wb_t.transpose().dot(R_wb_t_dt)) # w(t) = 1/dt * log(R_wb(t)^T * R_wb(t+dt))
#    w_world[i,:] = R_wb_t.dot(w_body[i,:]) # angular velocity from body frame to world frame
    v_world[i,:] = 1.0 / dt * (np.array(positions[i+stride])-np.array(positions[i]))
    v_body[i,:] = R_wb_t.transpose().dot(v_world[i,:])

plt.figure(1)
plt.plot(t[:-stride], w_body[:,0], label='wx (body)')
plt.plot(t[:-stride], w_body[:,1], label='wy')
plt.plot(t[:-stride], w_body[:,2], label='wz')
plt.legend()
plt.title('Angular velocity along trajectory')

plt.figure(2)
plt.plot(t[:-stride], v_world[:,0], label='vx (world)')
plt.plot(t[:-stride], v_world[:,1], label='vy')
plt.plot(t[:-stride], v_world[:,2], label='vz')
plt.legend()
plt.title('Linear velocity along trajectory')

cam = cameras.PinholeCamera(width, height, fx, fy, cx, cy)

interval_between_images = 24

write_motion_field = False
if write_motion_field:
    
    if not os.path.exists('%s/preview' % dataset_dir):
        os.makedirs('%s/preview' % dataset_dir)
    
    cur_id = 0
    for frame_id in range(1, len(image_data), interval_between_images):
        frame = image_data[frame_id]
        timestamp, img_path, pose = frame[:3]
        exr_img = OpenEXR.InputFile('%s/%s' % (dataset_dir, img_path))
        z = extract_depth(exr_img)
        
        #plt.figure(3)
        #plt.imshow(z)
        #plt.colorbar()
        #plt.title('Z-Depth')
        
        ang_vel = w_body[frame_id]
        lin_vel = v_body[frame_id]
        
        motion_field_x = np.zeros((height, width))
        motion_field_y = np.zeros((height, width))
        for u in range(width):
            for v in range(height):
                
                uc, vc = u-cx, v-cy
                Z = z[v][u]
                
                #x, y = uc/fx, vc/fy
                # interaction matrix for normalized pixel coordinates
                #B = np.array([[-1/Z, 0, x/Z, x*y, -(1+x*x), y],
                #               [0, -1/Z, y/Z, 1+y*y, -x*y, -x]])
                               
                B = np.array([[-fx/Z, 0, uc/Z, uc*vc/fx, -(fx**2 + uc**2)/fx, vc],
                               [0, -fy/Z, vc/Z, (fy**2 + vc**2)/fy, -uc*vc/fy, -uc]])
                
                motion_vec = B.dot(np.array([lin_vel[0], lin_vel[1], lin_vel[2], ang_vel[0], ang_vel[1], ang_vel[2]]))
                
                motion_field_x[v][u] = motion_vec[0]
                motion_field_y[v][u] = motion_vec[1]
                
#                motion_vec_numeric = motion_vector_numeric(cam, u, v, Z, frame_id, t, positions, quaternions)
        
        plt.figure(6)
        img = extract_grayscale(exr_img)
        plt.imshow(img, cmap = cm.Greys_r)
        
        s = 8 # space between motion field arrows, in pixels
        m = 2 # margin
        X, Y = np.meshgrid(np.arange(0, width, 1), np.arange(0, height, 1))
        plt.quiver(X[m:-m:s, m:-m:s], Y[m:-m:s, m:-m:s], motion_field_x[m:-m:s, m:-m:s], -motion_field_y[m:-m:s, m:-m:s],
                   color='g',
                   units='inches')
        plt.title('Motion field')
        plt.savefig('%s/preview/motion_field_%04d.png' % (dataset_dir, cur_id))
        plt.clf()
        
        cur_id += 1

'''
Command-line to stitch all these images in a video:

ffmpeg -f image2 -r 10 -i motion_field_%04d.png -y motion_field.mp4

'''