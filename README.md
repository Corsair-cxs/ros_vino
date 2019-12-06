# ros_vino

A generic ROS wrapper for OpenVINO, providing the following features:

- Support CPU, GPU and Myriad (Neural Compute Stick 2) platforms.
- Real-time SSD Object Detection
- Real-time Bounding Box Tracking

This package works perfectly with OpenVINO 2019 R3 Release (Newest Release as of Oct 2019).

## 1. Nodes

- [object_detection_ssd](src/object_detection_ssd): A Real-time SSD Object Detection using OpenVINO
  
  - Subscribe: 
    
    - A standard image topic of ```sensor_msgs::Image```, it can be from any USB camera or Realsense camera.
  - Publish: 
    - A new image topic of ```sensor_msgs::Image``` with bounding boxes
    
    - A list of detected object topic of ```ros_vino::Objects```
    
      
  
- [multi_kf_tracker](src/multi_kf_tracker): A Real-time Multi Tracker using Kalman Filter and Hungarian Algorithm

  - Subscribe: 
    - A standard image topic of ```sensor_msgs::Image```, it can be from any USB camera or Realsense camera.
    - A list of detected object topic of ```ros_vino::Objects```
  - Publish: 
    - A new image topic of ```sensor_msgs::Image``` with tracked bounding boxes
    - A list of tracked object topic of ```ros_vino::Objects```

## 2. Results

**MobileNet SSD Object Detection (Bottom)**: 30 FPS

**Multi Object Tracking (Upper)**: 30 FPS

![multi_kf_track](docs/multi_kf_track.gif)

## 3. Pre-requisites

1. Ubuntu 16.04/18.04
2. OpenVINO 2019 R3
3. ROS Kinetic / Melodic 

## 4. Installation

*This installation is only properly tested on Ubuntu 16.04, ROS Kinetic and OpenVINO 2019 R3.*

### 4.1 Install OpenVINO

1. Download OpenVINO from [here](https://software.intel.com/en-us/openvino-toolkit/choose-download/free-download-linux).

2. Install dependencies and OpenVINO.

   ```bash
   cd ~/Download
   tar xvf l_openvino_toolkit_<VERSION>.tgz
   cd l_openvino_toolkit_<VERSION>
   sudo -E ./install_openvino_dependencies.sh
   sudo -E ./install_GUI.sh
   ```

3. In the GUI, make sure the installation directory is set to `/opt/intel` by default.

4. Source OpenVINO environment variables.

   ```bash
   echo "source /opt/intel/openvino/bin/setupvars.sh" >> ~/.bashrc
   source ~/.bashrc
   ```

5. *Optional.* Setup Myriad for Neural Compute Stick 2.

   ```bash
   cd ~/Downloads
   cat <<EOF > 97-usbboot.rules
   SUBSYSTEM=="usb", ATTRS{idProduct}=="2150", ATTRS{idVendor}=="03e7", GROUP="users", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
   SUBSYSTEM=="usb", ATTRS{idProduct}=="2485", ATTRS{idVendor}=="03e7", GROUP="users", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
   SUBSYSTEM=="usb", ATTRS{idProduct}=="f63b", ATTRS{idVendor}=="03e7", GROUP="users", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
   EOF
   sudo cp 97-usbboot.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   sudo ldconfig
   rm 97-usbboot.rules
   ```

6. *Optional.* Install librealsense for Realsense Depth Camera D415/**D435**/D435i. Then install realsense-ros.

   ```bash
   sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
   sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
   sudo apt-get install librealsense2-dkms
   sudo apt-get install librealsense2-utils
   sudo apt-get install librealsense2-dev
   sudo apt-get install librealsense2-dbg
   
   cd ~/catkin_ws/src
   git clone https://github.com/intel-ros/realsense
   cd realsense
   git checkout 2.1.3
   ```

   

### 4.2 Install ros_vino

1. Clone this package into your workspace, assuming `~/catkin_ws` as your ROS workspace:

   ```bash
   cd ~/catkin_ws/src
   git clone http://github.com/songshan/ros_vino
   ```

2. Compile!

   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

   

## Object Detection SSD Demo

- Launch object detection demo with Intel Realsense:

  ```bash
  roslaunch ros_vino object_detection_ssd_realsense.launch
  ```

- Launch object detection demo with other camera:

  ```bash
  roslaunch ros_vino object_detection_ssd.launch
  ```

### Parameters

- **model_path** - Optional. Path to an .xml file with a trained model.
- **topic_image_input** - Optional. The topic name of input images.
- **topic_image_output** - Optional. The topic name of output images which contains bounding boxes.
- **score_threshold** - Optional. The minimum detection score to be considered as a detection. Default is 0.8.
- **device** - Optional. Specify the target device to infer on. Available choices are [CPU / GPU / MYRIAD]. Default is CPU.
- **bool_auto_resize** - Optional. Enables resizable input with support of ROI crop & auto resize. Default is true.
- **bool_pc** - Optional. Enables per-layer performance report. Default is false.
- **bool_raw** - Optional. Inference results as raw values. Default is false.



## Detection + Multi Tracker

- Launch object tracker demo with Intel Realsense and object detection:

  ```bash
  roslaunch ros_vino detection_track_demo.launch
  ```

### Parameters

- **topic_image_input** - Optional. The topic name of input images.
- **topic_objects_input** - Optional. The topic name of detected objects.
- **topic_image_output** - Optional. The topic name of output images which contains tracked bounding boxes.
- **topic_objects_output** - Optional. The topic name of tracked objects.
- **min_correspondence_cost** - Optional. The minimum correspondence cost to be considered as a new untracked detection. Default is 100.
- **p_sampling_time** - Optional. The output rate of object detection. Default is 30.0.
- **p_loop_rate** - Optional. The desired update rate of output image and objects. Default is 30.0.
- **p_tracking_duration** - Optional. The number of seconds to keep the tracked without a matched detection. Default is 0.5.
- **p_process_variance** - Optional. The process variance of Kalman filter. Default is 1.0.

- **p_process_rate_variance** - Optional. The process rate variance of Kalman filter. Default is 10.0.

- **p_measurement_variance** - Optional. The detection variance for Kalman filter. Default is 10000.0.



## Download your own model:

1. Download model through downloader:

   ```bash
   # Download Mobilenet-SSD Model using downloader
   cd /opt/intel/openvino/deployment_tools/open_model_zoo/tools/downloader
   sudo python3 ./downloader.py --name [MODEL_NAME]
   # Copy the downloaded model to ~/openvino_models/
   cd /opt/intel/openvino/deployment_tools/open_model_zoo/tools/downloader/public
   cp -r [MODEL_NAME]/ ~/openvino_models/models/FP16/[MODEL_NAME]
   ```

2. Optimize Caffe model

   ```bash
   sudo python3 /opt/intel/openvino/deployment_tools/model_optimizer/mo_caffe.py --input_model ~/openvino_models/models/FP16/[MODEL_NAME]/[MODEL_NAME].caffemodel --output_dir ~/openvino_models/ir/FP16/public/[MODEL_NAME]/ --mean_values [127.5,127.5,127.5] --scale_values [127.5]
   # Copy into ros_vino
   cp -r ~/openvino_models/ir/FP16/public/[MODEL_NAME]/ ~/catkin_ws/src/ros_vino/models/FP16/[MODEL_NAME]
   ```

3. Change "model_path" parameter to the model xml file in [launch file](launch/object_detection_ssd_realsense.launch).

   ```launch
   <param name="model_path" value="$(find ros_vino)/[PATH-TO-XML]" />
   ```

4. Launch!

   ```bash
   roslaunch ros_vino object_detection_ssd_realsense.launch
   ```

## Known Issues

- ```'Graph' object has no attribute 'node'``` error when optimizing model.
  - **Solution:** Run ```python3 -m pip install networkx==2.3```.

## Todo

- [ ] Write a new node for Object Detection Mask R-CNNs Segmentation in C++
- [ ] Write an easy install bash script to setup all installation.



## Notes

- You're welcome to push issue and I will help you to make this dedicated package to work on your machine too.
- You're welcome to suggest any Todo.
- Don't forget to give a star if you like this package! Thanks!=D