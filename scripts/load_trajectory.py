#!/usr/bin/env python

'''
Loads a camera trajectory from a file into Blender

Trajectory file should contain one line per frame containing the pose
(transformation from cam to world)

Line formatting is as follows:

frame_id tx ty tz qx qy qz qw

'''

import os
import sys
sys.path.append('/usr/lib/python3/dist-packages') # Add python3 packages to the environment
import mathutils
import bpy

cam = bpy.data.objects['Camera']
scene = bpy.data.scenes['Scene']

trajectory_path = os.environ['SYNTHESIZER_TRAJECTORY_PATH']

# Blender coordinate system is different than ours (z-axis is reversed)
R_blender_cam = mathutils.Matrix(((1, 0, 0),
    (0, -1, 0),
    (0, 0, -1)))
    
R_cam_blender = R_blender_cam

# Parse trajectory file
lines = [line.rstrip('\n') for line in open(trajectory_path)]
trajectory = {}
for line in lines:
    splitted = line.split(' ')
    frame_id = int(splitted[0])
    
    t_world_cam = [float(i) for i in splitted[1:4]]
    q = [float(i) for i in splitted[4:]]
    
    T = (mathutils.Quaternion([q[3], q[0], q[1], q[2]]).to_matrix()*R_cam_blender).to_4x4()
    T.translation = mathutils.Vector([t_world_cam[0], t_world_cam[1], t_world_cam[2]])
    
    trajectory[frame_id] = T

# Copy camera trajectory into current scene
cam.animation_data_clear()
scene.frame_start = min(trajectory.keys())
scene.frame_end = max(trajectory.keys())
    
for frame_id, T in trajectory.items():
    scene.frame_current = frame_id
    cam.matrix_world = T
    
    cam.keyframe_insert(data_path="location", frame=frame_id)
    cam.keyframe_insert(data_path="rotation_euler", frame=frame_id)
    
bpy.context.scene.frame_set(scene.frame_start)

