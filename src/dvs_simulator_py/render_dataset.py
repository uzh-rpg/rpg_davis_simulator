#!/usr/bin/env python

import rospy
import rospkg
import sys
import os.path
import subprocess

if __name__ == '__main__':

    rospy.init_node('render_dataset_node', anonymous=True)
    rospack = rospkg.RosPack()
    
    blender_path = '/usr/bin/blender'
    workspace_dir = rospack.get_path('dvs_simulator_py')
    dataset_dir = os.path.join(workspace_dir, 'datasets')
    dataset_name = rospy.get_param('dataset_name', 'untitled')
    camera_name = rospy.get_param('camera_name', 'Camera')

    scene_name = rospy.get_param('scene_name', '')
    if not scene_name:
        rospy.logfatal('No Blender scene name provided. Aborting.')
        sys.exit()
        
    scene_name += '.blend'
    
    trajectory_path = rospy.get_param('trajectory_path', '')

    python_scripts_cmd = []
    if os.path.isfile(trajectory_path):
        os.environ['SYNTHESIZER_TRAJECTORY_PATH'] = trajectory_path
        python_scripts_cmd += ['--python', os.path.join(workspace_dir, 'scripts', 'load_trajectory.py')]
    
    python_scripts_cmd += ['--python', os.path.join(workspace_dir, 'scripts', 'prepare_dataset.py')]
    
    os.environ['SYNTHESIZER_DATASET_DIR'] = dataset_dir
    os.environ['SYNTHESIZER_DATASET_NAME'] = dataset_name
    os.environ['SYNTHESIZER_CAMERA_NAME'] = camera_name
    
    blender_cmd = [blender_path, 
              '-b', os.path.join(dataset_dir, 'scenes', scene_name)] + python_scripts_cmd + ['-a']

    try:
        retcode = subprocess.call(blender_cmd)
        if retcode < 0:
            print >>sys.stderr, "Child was terminated by signal", -retcode
        else:
            print('Shutting down the Blender instance')
    
    except OSError, e:
        print('The Blender instance has died!')
        print >>sys.stderr, "Execution failed:", e