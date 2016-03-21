#!/usr/bin/env python

import rospy
import rosbag
import numpy as np
from dvs_msgs.msg import EventArray
import math

from dvs_simulator_py import dataset_utils
from dvs_simulator_py import dvs_simulator
from scipy.interpolate import RectBivariateSpline
import scipy.ndimage
from matplotlib import pyplot as plt
from math import fabs


#%% Load HD map

plt.close('all')

img_path = '/home/henri/Dev/catkin_ws/src/rpg_blender/blender_synthesizer/textures/virtual_flying_room/mountain.jpg'
#img_path = '/home/henri/Downloads/zebra_stripes_blurred.jpg'
map_img = scipy.ndimage.imread(img_path, flatten=True) / 255.0

#%% Prepare sliding trajectory of the DVS over the map

dvs_size = (32, 32) # (height, width)
X, Y = np.meshgrid(np.arange(0, map_img.shape[1], 1), np.arange(0, map_img.shape[0], 1)) 

use_log = True

if use_log:
    map_interpolator = RectBivariateSpline(Y[:,0], X[0,:], dataset_utils.safe_log(map_img))
else:
    map_interpolator = RectBivariateSpline(Y[:,0], X[0,:], map_img)

init_pos = (600, 650) # top-left corner of the DVS
end_pos = (610, 640)
#init_pos = (70, 38) # top-left corner of the DVS
#end_pos = (125, 68)

num_steps = 50

trajectory = np.vstack([np.linspace(init_pos[0], end_pos[0], num_steps),
                        np.linspace(init_pos[1], end_pos[1], num_steps)]).T
                        

#%% Go along the trajectory and get the observed image in the sensor

x,y = trajectory[0]
sensor_x = np.arange(x, x + dvs_size[1])
sensor_y = np.arange(y, y + dvs_size[0])
init_sensor = map_interpolator(sensor_y, sensor_x)

C = 0.1

cur_time = 0.0
last_pub_event_timestamp = 0.0

#bag = rosbag.Bag('/home/henri/Downloads/bag.bag', 'w')

event_framerate = 200.0 # event packets per second

reconstructed_sensor = init_sensor.copy()

sim = dvs_simulator.DvsSimulator(cur_time, init_sensor, C)

events = []
delta_t = 0.010

u,v = 18, 16 # focus on I(t) for a specific pixel
I_t = []

for x,y in trajectory[1:]:
    
    cur_time += delta_t
    
    print cur_time
    
    sensor_x = np.linspace(x, x + dvs_size[1], endpoint=False, num=dvs_size[1])
    sensor_y = np.linspace(y, y + dvs_size[0], endpoint=False, num=dvs_size[0])
    current_sensor = map_interpolator(sensor_y, sensor_x)
    
    I_t.append([cur_time, current_sensor[v,u]])
    
    current_events = sim.update(cur_time, current_sensor)
    events += current_events
 
events = sorted(events, key=lambda e: e.ts)
events_at_uv = []
for e in events:
    pol = +1 if e.polarity else -1

    if fabs(e.x-u) < 0.01 and fabs(e.y-v) < 0.01:
        events_at_uv.append([e.ts.to_sec(), e.x, e.y, pol])    
    
    reconstructed_sensor[e.y,e.x] += pol * C
    
#bag.close()


#%%

plt.figure()
plt.subplot(141)
plt.imshow(init_sensor, interpolation='none', cmap='gray')
plt.title('Reference intensity')
plt.subplot(142)
plt.imshow(current_sensor, interpolation='none', cmap='gray')
plt.title('Final intensity')
plt.subplot(143)
plt.imshow(reconstructed_sensor, interpolation='none', cmap='gray')
plt.title('Reconstructed intensity')
plt.subplot(144)
diff = reconstructed_sensor - current_sensor
plt.imshow(reconstructed_sensor-current_sensor, interpolation='none')
plt.title('Difference')
plt.colorbar()

#%% 

I_t = np.array(I_t)
events_at_uv = np.array(events_at_uv)

zero_contrast_level = I_t[0,1]
reconstructed_I_t = np.cumsum(C*events_at_uv[:,3]) + zero_contrast_level

plt.figure()
plt.plot(I_t[:,0], I_t[:,1], 'r')
plt.plot(events_at_uv[:,0], reconstructed_I_t, 'b+')

npos = np.ceil((np.max(I_t[:,1])-zero_contrast_level)/C)+1
nneg = np.ceil((zero_contrast_level-np.min(I_t[:,1]))/C)+1
for n in range(int(npos)):
    plt.axhline(y=zero_contrast_level+n*C, color='red', linestyle='dotted')
for n in range(int(nneg)):
    plt.axhline(y=zero_contrast_level-n*C, color='red', linestyle='dotted')
for pt in I_t[:,0]:
    plt.axvline(x=pt, color='green', linestyle='dotted')

plt.xlabel('t (s)')
plt.ylabel('log I')
plt.title('Intensity vs time for pixel (%d,%d)' % (u,v))