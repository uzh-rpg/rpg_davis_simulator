#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 19 17:58:47 2016

Load some synthetic dataset and display the motion field

@author: Henri Rebecq
"""

import rospkg
import OpenEXR
import numpy as np
from vikit_py import transformations as transformations
import os.path
from matplotlib import pyplot as plt
from matplotlib import cm as cm
from dvs_simulator_py import dataset_utils
    
  
""" Compute motion vector analytically (using the interaction/feature sensitivity matrix) """
def motion_vector_analytic(cam, u, v, Z, lin_vel, ang_vel):
    fx, fy = cam[2], cam[3]
    cx, cy = cam[4], cam[5]
    uc, vc = u-cx, v-cy     
     
    B = np.array([[-fx/Z, 0, uc/Z, uc*vc/fx, -(fx**2 + uc**2)/fx, vc],
                   [0, -fy/Z, vc/Z, (fy**2 + vc**2)/fy, -uc*vc/fy, -uc]])
    
    return B.dot(np.hstack((lin_vel, ang_vel))) 


""" Compute motion field given Z-depth map, camera calibration and linear/angular velocities """
def compute_motion_field(cam, z, lin_vel, ang_vel):
    motion_field = np.zeros(z.shape + (2,), np.float64)
    for v in range(z.shape[0]):
        for u in range(z.shape[1]):
            Z = z[v,u]
            motion_field[v,u,:] = motion_vector_analytic(cam, u, v, Z, lin_vel, ang_vel)
    return motion_field



def plot_velocities(t, v_world, w_body, stride):
    plt.figure(figsize=(6,8))
    plt.subplot(2,1,1)
    plt.plot(t[:-stride], v_world[:,0], label='vx')
    plt.plot(t[:-stride], v_world[:,1], label='vy')
    plt.plot(t[:-stride], v_world[:,2], label='vz')
    plt.ylabel('Linear velocity (world frame), m/s')
    plt.legend()
    
    plt.subplot(2,1,2)
    plt.plot(t[:-stride], w_body[:,0], label='wx')
    plt.plot(t[:-stride], w_body[:,1], label='wy')
    plt.plot(t[:-stride], w_body[:,2], label='wz')
    plt.ylabel('Angular velocity (body frame), rad/s')
    plt.xlabel('Time (s)')
    plt.legend()
    


if __name__ == '__main__':    

    rospack = rospkg.RosPack()
    plt.close('all')
    
    """ User-defined parameters """
    dataset_name = 'dunes_translation'
    interval_between_images = 48
    write_motion_field = True
    check_analytic_motion_field = False
    
    dataset_dir = os.path.join(rospack.get_path('rpg_datasets'), 'DVS', 'synthetic', 'full_datasets', dataset_name, 'data')
    
    t, img_paths, positions, orientations, cam = dataset_utils.parse_dataset(dataset_dir)
    
    width, height = cam.width, cam.height
    fx, fy = cam.focal_length
    cx, cy = cam.principle_point
    
    stride = 1
    v_world, v_body, w_body = dataset_utils.linear_angular_velocity(t, positions, orientations, stride)
    
    plot_velocities(t, v_world, w_body, stride)
    
    if write_motion_field:
        
        if not os.path.exists('%s/preview' % dataset_dir):
            os.makedirs('%s/preview' % dataset_dir)
        
        s = 8 # space between motion field arrows, in pixels
        m = 2 # margin
        X, Y = np.meshgrid(np.arange(0, width, 1), np.arange(0, height, 1))    
        
        fig = plt.figure()
        ax = fig.add_subplot(111, title='Motion field')
    
        for frame_id in range(1, len(t), interval_between_images):
            
            exr_img = OpenEXR.InputFile('%s/%s' % (dataset_dir, img_paths[frame_id]))
            z = dataset_utils.extract_depth(exr_img)
    
            lin_vel = v_body[frame_id]        
            ang_vel = w_body[frame_id]
            
            motion_field = np.zeros((height, width, 2))
                
            for u in range(width):
                for v in range(height):
                    Z = z[v,u]                            
                    motion_field[v,u,:] = motion_vector_analytic(cam, u, v, Z, lin_vel, ang_vel)
            
            img = dataset_utils.extract_grayscale(exr_img)
            ax.imshow(img, cmap = cm.Greys_r, interpolation='none')
            
            ax.quiver(X[m:-m:s, m:-m:s], Y[m:-m:s, m:-m:s], motion_field[m:-m:s, m:-m:s, 0], -motion_field[m:-m:s, m:-m:s, 1],
                       color='g',
                       units='inches')
                       
            fig.savefig('%s/preview/motion_field_%04d.png' % (dataset_dir, frame_id // interval_between_images))
            ax.cla()
            