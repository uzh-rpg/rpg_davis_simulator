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
    fx, fy = cam.focal_length
    cx, cy = cam.principle_point
    uc, vc = u-cx, v-cy     
     
    B = np.array([[-fx/Z, 0, uc/Z, uc*vc/fx, -(fx**2 + uc**2)/fx, vc],
                   [0, -fy/Z, vc/Z, (fy**2 + vc**2)/fy, -uc*vc/fy, -uc]])
    
    return B.dot(np.hstack((lin_vel, ang_vel))) 



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
    dataset_name = 'one_textured_plane_helicoidal'
    interval_between_images = 24
    write_motion_field = True
    
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
                    #motion_field[v,u,:] = motion_vector_numeric(cam, u, v, Z, frame_id, t, positions, quaternions)
            
            img = dataset_utils.extract_grayscale(exr_img)
            ax.imshow(img, cmap = cm.Greys_r)
            
            ax.quiver(X[m:-m:s, m:-m:s], Y[m:-m:s, m:-m:s], motion_field[m:-m:s, m:-m:s, 0], -motion_field[m:-m:s, m:-m:s, 1],
                       color='g',
                       units='inches')
                       
            fig.savefig('%s/preview/motion_field_%04d.png' % (dataset_dir, frame_id // interval_between_images))
            ax.cla()
    
    '''
    Command-line to stitch all these images in a video:
    
    ffmpeg -f image2 -r 10 -i motion_field_%04d.png -y motion_field.mp4
    
    '''