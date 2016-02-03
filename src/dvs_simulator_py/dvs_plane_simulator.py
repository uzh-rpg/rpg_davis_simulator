#!/usr/bin/env python

import rospy
import rospkg
import rosbag
import OpenEXR
import numpy as np
import os.path
import time
import cv2
from dvs_msgs.msg import Event, EventArray
import math

from dvs_simulator_py import dvs_simulator
from dvs_simulator_py import dataset_utils
from scipy.interpolate import RectBivariateSpline
import scipy.ndimage
from matplotlib import pyplot as plt

#%% Load HD map

plt.close('all')

#img_path = '/home/henri/Dev/catkin_ws/src/rpg_blender/blender_synthesizer/textures/virtual_flying_room/road.jpg'
img_path = '/home/henri/Dev/catkin_ws/src/rpg_blender/blender_synthesizer/textures/virtual_flying_room/mountain.jpg'
map_img = scipy.ndimage.imread(img_path, flatten=True) / 255.0

#%%
#plt.figure()
#plt.imshow(map_img)

#%% Prepare sliding trajectory of the DVS over the map

dvs_size = (128, 128) # (height, width)
X, Y = np.meshgrid(np.arange(0, map_img.shape[1], 1), np.arange(0, map_img.shape[0], 1)) 

use_log = True

if use_log:
    map_interpolator = RectBivariateSpline(Y[:,0], X[0,:], dvs_simulator.safe_log(map_img))
else:
    map_interpolator = RectBivariateSpline(Y[:,0], X[0,:], map_img)

init_pos = (600, 650) # top-left corner of the DVS
end_pos = (615, 615)

#init_pos = (2000, 1000) # top-left corner of the DVS
#end_pos = (2015, 1015)

#init_pos = (1092, 1343) # top-left corner of the DVS
#end_pos = (1092+100, 1343)

num_steps = 150

trajectory = np.vstack([np.linspace(init_pos[0], end_pos[0], num_steps),
                        np.linspace(init_pos[1], end_pos[1], num_steps)]).T
                        

#%% Go along the trajectory and get the observed image in the sensor

x,y = trajectory[0]
sensor_x = np.arange(x, x + dvs_size[1])
sensor_y = np.arange(y, y + dvs_size[0])
init_sensor = map_interpolator(sensor_y, sensor_x)

C = 0.15

cur_time = 0.0
last_pub_event_timestamp = 0.0

bag = rosbag.Bag('/home/henri/Downloads/bag.bag', 'w')

event_framerate = 200.0 # event packets per second

reconstructed_sensor = init_sensor.copy()
reference_values = init_sensor.copy()
It_array = init_sensor.copy()

events = []
delta_t = 0.010
for x,y in trajectory[1:]:
    
    cur_time += delta_t
    
    print cur_time

    # publish events
    if cur_time - last_pub_event_timestamp > 1.0/event_framerate:
        event_array = EventArray()
        event_array.header.stamp = rospy.Time(secs=cur_time)
        event_array.width = dvs_size[1]
        event_array.height = dvs_size[0]
        event_array.events = events
        
        bag.write(topic='/dvs/events', msg=event_array, t=rospy.Time(secs=cur_time))
        
        events = []
        last_pub_event_timestamp = cur_time
    
    
    sensor_x = np.linspace(x, x + dvs_size[1], endpoint=False, num=dvs_size[1])
    sensor_y = np.linspace(y, y + dvs_size[0], endpoint=False, num=dvs_size[0])
    current_sensor = map_interpolator(sensor_y, sensor_x)
    
    # for each pixel, generate events
    current_events = []
    for u in range(dvs_size[1]):
        for v in range(dvs_size[0]):
            events_for_px = []
            t_dt = cur_time
            t = cur_time - delta_t
            It = It_array[v,u]
            It_dt = current_sensor[v,u]
            previous_crossing = reference_values[v,u]
            
            
            tol = 0.0001
            if math.fabs(It-It_dt) > tol: 
                pol = (It_dt >= It)
                
                list_crossings = []
                all_crossings_found = False
                cur_crossing = previous_crossing
                while not all_crossings_found:
                    if pol:
                        cur_crossing = cur_crossing + C
                        if cur_crossing > It and cur_crossing <= It_dt:
                            list_crossings.append(cur_crossing)
                        else:
                            all_crossings_found = True
                    else:
                        cur_crossing = cur_crossing - C
                        if cur_crossing < It and cur_crossing >= It_dt:
                            list_crossings.append(cur_crossing)
                        else:
                            all_crossings_found = True
                            
                for crossing in list_crossings:            
                    if(pol):
                        assert(crossing > It and crossing <= It_dt)
                    else:
                        assert(crossing < It and crossing >= It_dt)
                    
                    te = t + (crossing-It)*delta_t/(It_dt-It)
                    events_for_px.append(dvs_simulator.make_event(u,v,te,pol))
                    
                current_events += events_for_px
                    
                if bool(list_crossings):
                    if pol:
                        reference_values[v,u] = max(list_crossings)
                    else:
                        reference_values[v,u] = min(list_crossings)
    
    It_array = current_sensor.copy()
    events = events + current_events
    
    for e in current_events:
        if e.polarity:
            pol = +1
        else:
            pol = -1
        reconstructed_sensor[e.y,e.x] += pol * C
    
bag.close()


#%%

plt.figure()
plt.subplot(141)
plt.imshow(init_sensor, interpolation='none')
plt.title('Reference intensity')
plt.subplot(142)
plt.imshow(current_sensor, interpolation='none')
plt.title('Final intensity')
plt.subplot(143)
plt.imshow(reconstructed_sensor, interpolation='none')
plt.title('Reconstructed intensity')
plt.subplot(144)
diff = reconstructed_sensor - current_sensor
plt.imshow(reconstructed_sensor-current_sensor, interpolation='none')
plt.title('Difference')
plt.colorbar()