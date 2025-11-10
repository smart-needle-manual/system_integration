# Setup and Launch, Shape-Sensing Stylet: 1R01EB036015-01 
## Project Personnel
### *Principal Investigators*: Junichi Tokuda, PhD; Iulian Iordachita, PhD; Robert Cormack, PhD
### *Author*: Rajdeep Banerjee, MEng, Research Trainee - Tokuda Lab, Department of Radiology @ Brigham and Women's Hospital, 75 Francis St, Boston, MA, 02115
### *Supervisor*: Mariana Bernardes, PhD
### *Current & Past Contributors*: 
Johns Hopkins University: Dimitri Lezcano, PhD; Jin Seob Kim, PhD; Jacynthe Francoeur, MS; Yinsong Ma, MS; Kayleigh Huk 
Brigham & Women's Hospital: Pedro Moreira, PhD; Nobuhiko Hata, PhD; Lori Foley; Kemal Tuncali, MD; Clare Tempany, MD, PhD
## Goal



Gynecologic malignancies accounted for over 114,810 new cancer cases and approximately 35,640
deaths in 2023 in the United States. Brachytherapy has been used to treat locally advanced cervical and
endometrial cancers since the early 20th century and is now part of the standard of care. Brachytherapy involves
the precise placement of short-range radioactive sources near or in direct contact with the tumor through thin
catheters, enabling high radiation doses in the target volume with rapid fall-off to protect adjacent normal
structures. Today, high-dose-rate (HDR) brachytherapy is commonly employed along with a computer-controlled
remote afterloading system, which allows accurate control of radiation dose for each catheter by adjusting the
“dwell time.” Though computer-controlled afterloading systems are widespread, the quality of brachytherapy is
still limited by suboptimal catheter placement. Clinicians often struggle to properly deploy the catheters in the
target volume because of the deviation of the catheters from the intended path during insertion and lack of
quantitative catheter position feedback or the dosimetric consequences resulting from the current catheter
locations. Intraoperative imaging, either computed tomography (CT) or magnetic resonance (MR), potentially
provides such feedback and allows for “adaptive catheter placement,” where the clinician adjusts catheter
location until optimal dosimetry is achieved. However, adaptive catheter placement is not practical in the current
form because it requires iterative implantation and imaging; each iteration involves positioning of the patient for
imaging and catheter placement and moving of the clinician between the imaging room and the control room. To
enable adaptive catheter placement in a wide range of clinical settings, we will develop a catheter placement
manipulator system that combines (1) a state-of-the-art fiber-optic shape-sensing stylet to obtain real-time
quantitative measurement of the catheter trajectories in the patient, and (2) a teleoperated catheter placement
manipulator to shorten the turnaround time for catheter placement and evaluation, (3) a visualization framework
that provides quantitative measures of a catheter’s deviation from its intended trajectory and real-time evaluation
of the consequences to the achievable radiation dose distribution. We hypothesize that the combination of real-
time catheter trajectory digitization and quick catheter insertion will allow adaptive catheter placement, where
the catheter locations are optimized through frequent iteration of the plan-insert-check cycle with real-time
quantitative dosimetry feedback, leading to optimal radiation dose distribution. We will pursue the following
specific aims: (Aim 1) Develop a fiber-optic shape-sensing stylet for real-time catheter tracking to achieve real-
time tracking and prediction of the catheter trajectory for real-time feedback; (Aim 2) Develop a teleoperated
catheter insertion manipulator for quick catheter placement to achieve a shorter turnaround time for the frequent
plan-insert-check cycle; (Aim 3) Develop, optimize, and validate the system for teleoperated adaptive catheter
placement to test the impact of teleoperated adaptive catheter placement for optimal dose distribution.

This letter addresses the targeting challenges in MRI-guided transperineal needle placement for prostate cancer (PCa) diagnosis and treatment, a procedure where accuracy is crucial for effective outcomes. We introduce a parameter-agnostic trajectory correction approach incorporating a data-driven closed-loop strategy by radial displacement and an FBG-based shape sensing to enable autonomous needle steering. In an animal study designed to emulate clinical complexity and assess MRI compatibility through a PCa mock biopsy procedure, our approach demonstrated a significant improvement in targeting accuracy (p < 0.05), with mean target error of only 2.2 ± 1.9 mm on first insertion attempts, without needle reinsertions. To the best of our knowledge, this work represents the first in vivo evaluation of robotic needle steering with FBG-sensor feedback, marking a significant step towards its clinical translation.

## Setup






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
