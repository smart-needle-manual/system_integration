# Setup and Launch, Shape-Sensing Stylet: 1R01EB036015-01 
## Project Personnel
### *Principal Investigators*: Junichi Tokuda, PhD; Iulian Iordachita, PhD; Robert Cormack, PhD
### *Author*: Rajdeep Banerjee, MEng, Research Trainee - Tokuda Lab, Department of Radiology @ Brigham and Women's Hospital, 75 Francis St, Boston, MA, 02115
### *Supervisor*: Mariana Bernardes, PhD
### *Current & Past Contributors*: 
### Johns Hopkins University: Dimitri Lezcano, PhD; Jin Seob Kim, PhD; Jacynthe Francoeur, MS; Yinsong Ma, MS; Kayleigh Huk 
### Brigham & Women's Hospital: Pedro Moreira, PhD; Nobuhiko Hata, PhD; Kemal Tuncali, MD; Lori Foley, CVT; Clare Tempany, MD, PhD
## Background
### > The current standard of care in high-dose-rate (HDR) brachytherapy uses a catheter control by a computer-based remote afterloading system to deliver localized radiation to precisely target and eliminate cancerous tissue in cases such as prostate and gyencologic cancer. 
### > This approach necessitates frequent repositioning of the catheter due to lack of real-time feedback and deflections from contact with stiff tumors; intraoperative imaging, such as with CT or MRI, is untenable due to the neeed for frequent transitions involving patients and clinial staff. 
### > Previous work by membrs of this team [1, 2] showed that an approach leveraging fiber-optic shape-sensing, a robotic guide, and a quantiative image analysis framework can track deviations from intended trajectory and spatial dose distribution. 
### > The most recent and current working iteration of this approach uses one-dimensional Cosserat models of inextensible, elastic needle deformation using Lie group algebras to mdeol needle-tissue biomechanical interactions. 
### > However, while the approcah achieves sub-millimeter accuracy in deflection optimization, the use of a robotic guide and several nodal components in Slicer-ROS, the open-source visualization framework, reduces computational speed, perhaps without significantly improving prediction accuracy for position of the needle tip and downstream dosimetric calculations.

#### 1: https://ieeexplore.ieee.org/abstract/document/10669207
#### 2: https://ieeexplore.ieee.org/abstract/document/10801886

## Goal
### The purpose of the work delineated here is thus to 
### (1) remove the dependence of the system on robotic guidance
### (2) remove the OpenIGTLink bridge currently used for ROS-Slicer communication
### (3) validate the accurcay of the updated workflow. 

Degrees of Freedom in Plan-Insert-Check Cycle

## Setup
### Make sure to install and source ros2 humble in your workspace by following the instructions outlined here (https://docs.ros.org/en/humble/Installation.html and https://docs.ros.org/en/humble/Tutorials.html) and here (https://docs.ros.org/en/humble/Installation.html and https://docs.ros.org/en/humble/Tutorials.html) 





###### NOTE FOR BELOW: OpenIGT and ros2 humble already set up in obgynbrachy ########
###### NOTE: Tokuda lab members using AMIGO Gadgetron, follow Ubuntu directions ALWAYS ######
##### Build OpenIGT (needed for now) from https://openigtlink.org/library/build #####
##### For ROS 2 Humble installation and instructions, see: https://docs.ros.org/en/humble/Installation.html and https://docs.ros.org/en/humble/Tutorials.html #####

################### Launch Directions ###########################################################

# Author: Rajdeep Banerjee, MEng, Research Trainee in Brigham and Women's Hospital Radiology Department, Tokuda Lab, 75 Francis St, Boston, MA, 02115

# Supervisors: Dr. Mariana Bernardes (Direct) and Dr. Junichi Tokuda (PI)

########### Package branches TODO: Make working version main ###########################
~$ mkdir -p <dir_name>/src
~$ cd <dir_name>/src
~/<dir_name>/src$ git clone -b slicer_manual_mod https://github.com/smart-needle-manual/ros2_igtl_bridge.git
~/<dir_name>/src$ git clone -b ros_param_manual_trigger https://github.com/smart-needle-manual/ros2_needle_shape_publisher.git
~/<dir_name>/src$ git clone https://github.com/smart-needle-manual/ros2_hyperion_interrogator.git
~/<dir_name>/src$ git clone https://github.com/smart-needle-manual/3DoFSmartTemplate-ROS.git
~/<dir_name>/src$ git clone -b slicer_mod_manual https://github.com/smart-needle-manual/trajcontrol_jhu.git

#Turn ON hyperion interrogator black box. Blue light should be emitting from black box, and blue blinking should be coming from right side of PC (face the monitor)
#DO NOT forget to test Hyperion Interrogator. If this is not working, ensure that optical and computer connections are secure, that ens1f1 is set to Hyperion, and that ens1f1 Hyperion is set to 10.0.0.56 in Settings (see top right of computer screen). The idea is: same network, different IP (n+1 rule).  
#New terminal
cd ..
cd ..
cd HyperionInterrogator
pyton3 plotFBGSpectra.py 10.0.0.55
#Prepare to run launch files for needle shape sensing
cd smart_needle_manual
cd src
# If build not done (remove build, install, log, and build from src): colcon build --cmake-args -DOpenIGTLink_DIRR:PATH:=~/OpenIGTLink-build
#In a new terminal
. install/setup.bash
ros2 launch trajcontrol jhu_needle.launch.py manual_mode:=True
#In a new terminal
. install/setup.bash
ros2 run hyperion_interrogator calibrate_sensors --ros-args -r __ns:=/needle
#Hit Slicer settings like icon in ~/Slicer-5.8.1-linux-amd64$
#In a new terminal (ideal)
ros2 launch ros2_igtl_bridge bridge.launch.py
#Use dropdown menu in Slicer to go to IGT > SmartNeedle
#Create new Linear Transform
Start Client
#In a new terminal (best). This default command should work.
ros2 topic pub /stage/state/needle_pose geometry_msgs/PoseStamped "{header: {frame_id: 'needle'}, pose: {position: {x: 0.0, y: 0.0, z: 200.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" & ros2 topic pub /needle/state/skin_entry geometry_msgs/Point "{x: 0.0, y: 0.0, z: 0.0 }" & wait
#Deform the needle (gently, at tip not mroe than 20 degrees to preserve elastic deformation). WAIT 20 SECONDS ON FIRST DEFORMATION---it takes some time. 
################################################### Do not panic. All files are available on github if issues arise with modifications. #####################################
