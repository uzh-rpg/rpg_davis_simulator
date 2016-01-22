#!/usr/bin/env python

import os
import os.path
import sys
sys.path.append('/usr/lib/python3/dist-packages') # Add python3 packages to the environment
import mathutils
import bpy
from math import *

def fov_to_focal(width, angle_rad):
    f = 0.5 * width / tan(0.5 * angle_rad)
    return f

def goto_frame(scene, k):
    scene.frame_current = k
    bpy.context.scene.frame_set(bpy.context.scene.frame_current)
    

dataset_dir = os.environ['SYNTHESIZER_DATASET_DIR']
dataset_name = os.environ['SYNTHESIZER_DATASET_NAME']

cam = bpy.data.objects['Camera']
scene = bpy.data.scenes['Scene']

data_dir = os.path.join(dataset_dir, 'full_datasets', dataset_name, 'data')

scene.render.filepath = '%s/exr/' % data_dir
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = 'OPEN_EXR'
scene.render.image_settings.exr_codec = 'PIZ'
scene.render.image_settings.color_depth = '32'
scene.render.image_settings.color_mode = 'RGB'
scene.render.image_settings.use_zbuffer = True

first_frame = scene.frame_start
last_frame = scene.frame_end
framerate = 1000.0 # fps

# Blender coordinate system is different than ours
T_blender_cam = mathutils.Matrix(((1, 0, 0, 0),
    (0, -1, 0, 0),
    (0, 0, -1, 0),
    (0.0, 0.0, 0.0, 1.0)))
    
T_cam_blender = T_blender_cam

if not os.path.exists(data_dir):
    os.makedirs(data_dir)
    
if not os.path.exists('%s/exr' % data_dir):
    os.makedirs('%s/exr' % data_dir)
    
# Write camera information to a YAML file
cam_file = open('%s/camera.yaml' % data_dir, 'w')
cam_data = {}
cam_data['cam_width'] = scene.render.resolution_x
cam_data['cam_height'] = scene.render.resolution_y
cam_data['cam_fx'] = fov_to_focal(scene.render.resolution_x, cam.data.angle)
cam_data['cam_fy'] = fov_to_focal(scene.render.resolution_x, cam.data.angle)
cam_data['cam_cx'] = scene.render.resolution_x / 2.0
cam_data['cam_cy'] = scene.render.resolution_y / 2.0

yaml.dump(cam_data, cam_file)

# Write images.txt and trajectory.txt
images_file = open('%s/images.txt' % data_dir, 'w')
groundtruth_file = open('%s/trajectory.txt' % data_dir, 'w')
info_file = open('%s/info.txt' % data_dir, 'w')

timestamp = 0.0
for k in range(first_frame, last_frame+1):

    goto_frame(scene, k)
    timestamp += 1.0 / framerate
    
    T_world_camblender = cam.matrix_world.copy()
    T_world_cam = T_world_camblender * T_blender_cam
    
    T = (T_world_cam).translation
    q = (T_world_cam).to_quaternion()
    groundtruth_file.write('%d %f %f %f %f %f %f %f\n' % (k, T[0], T[1], T[2], q[1], q[2], q[3], q[0]))
    images_file.write('%d %06f exr/%04d.exr\n' % (k, timestamp, k))

# Write information about Blender configuration in info file
scene_name = bpy.path.basename(bpy.context.blend_data.filepath)
scene_name = scene_name.split('.')[0]

info_file.write('Blender scene: %s\n' % scene_name)
info_file.write('Render engine: %s\n' % scene.render.engine)

images_file.close()
groundtruth_file.close()
info_file.close()
