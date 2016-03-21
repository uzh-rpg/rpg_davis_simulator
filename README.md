# DAVIS simulator

## How does it work?

                render_dataset.py                dvs_simulator.py
Blender scene --------------------> DVS dataset -----------------> Rosbag
                Geometry, Motion                    DVS params

### Technical details

A DVS dataset is generated offline by a rendering engine (Blender in our case).
It contains a large number of images, obtained by raytracing along the camera trajectory with a very fine temporal resolution).

The data is stored as OpenEXR files, which contain the image data and depth maps as floating-point values in channels ('R', 'G', 'B'), 'Z' respectively.

*Note:* The intensity images are encoded in the linear color space (as opposed to sRGB color space). This is because the DVS work directly with irradiance values. It's important that the intensity images are encoded with floating-point accuracy to simulate correctly the high dynamic range of the DVS.

## What does the DAVIS simulator output?

- /dvs/events: event stream
- /dvs/camera_info: camera calibration
- /dvs/event_triggering_info: information about the DVS (contrast thresholds used, mostly)
- /dvs/pose: groundtruth transformation T_w_cam (transforms points from camera frame to world frame)
- /dvs/twist: linear and angular velocities, expressed in the camera coordinate frame
- /dvs/image_raw: intensity image (grayscale, 8bits)
- /dvs/depth_map: depth map (32bits floating-point)

## Compilation

    cd $path_to_your_catkin_ws$/src
    
Checkout the necessary dependencies:

- rpg_dvs_ros (https://github.com/uzh-rpg/rpg_dvs_ros)

        git clone https://github.com/uzh-rpg/rpg_dvs_ros
        git checkout -t origin/develop
        
- rpg_datasets (https://github.com/uzh-rpg/rpg_datasets)

        git clone https://github.com/uzh-rpg/rpg_datasets

Build the necessary packages:

    catkin build dvs_msgs rpg_datasets
    
Source your catkin workspace:

    source $path_to_your_catkin_ws$/devel/setup.bash

## FAQ

### "I just want a DVS dataset with groundtruth and standard parameters for the DVS"

Just use directly the rosbags. They contain the event stream, along with the groundtruth poses, raw images (DAVIS) and depth maps

### "I want to use an existing scene but be able to change the DVS parameters (contrast threshold, noise, etc.)"

You need to download a full DVS dataset, and then roslaunch the dvs_simulator (dataset path and DVS parameters are set in the launch file).

### "Where do I find datasets?"

- Rosbags: in the [rpg_datasets]() repository (/dvs/synthetic/rosbags/)
- Full DVS datasets: in the [rpg_datasets]() repository ('/dvs/synthetic/full_datasets/)
- Blender scenes: in the [rpg_datasets]() repository ('/dvs/synthetic/scenes/)

### "I want to change the [camera trajectory | camera calibration | scene geometry | textures]"

You need to change this directly in the Blender scene and then render the dataset again using render_dataset.py to produce a full dataset. You will then run the simulator on this new full dataset.

*Note:* you can provide the camera trajectory to render_dataset.py as a text file (with the argument 'trajectory_path' in the launch file)

### "I want to create a new DVS dataset"

Just create your scene into Blender and use render_dataset.py to render it.
Animate the camera using the standard Blender tools (keyframes, trajectory modifiers, etc.). You can also load a camera trajectory from a text file.

Note that there are a small number of requirements for the Blender synthesizer to work properly:

- the scene must contain *one* (and only one) camera named 'Camera'
- the camera used must be 'Perspective' (other types are not supported yet)
- 'First frame' and 'Last frame' values need to be set

### "What do the depthmaps contain?"

Depthmaps are encoded as 32-bit floating-point values.
The value encoded in the depth map depends on the render engine used by Blender.

- 'Blender Render' -> Depth along the optical axis (= Z-depth, fronto-parallel)
- 'Cycles' -> Euclidean depth (depth along the optical ray, for each pixel)

## Roadmap

- Model the sensor more accurately (noise spectral density, hot pixels/regions, pixel refractory period, etc.)
- Add an IMU simulator (with adjustable noise + bias)
- Include some example code (Python, C++) to parse data from the generated rosbags

