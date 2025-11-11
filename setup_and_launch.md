# Setup and Launch, Shape-Sensing Stylet: 1R01EB036015-01
**Principal Investigators**
>**Junichi Tokuda**, PhD (BWH); **Iulian Iordachita**, PhD (JHU); **Robert Cormack**, PhD (BWH)

**Author**
>**Rajdeep Banerjee**, MEng - Research Trainee @ Tokuda Lab, Brigham and Women's Hospital, 75 Francis St, Boston, MA, 02115

**Supervisor**
>**Mariana Bernardes**, PhD

**Current & Past Contributors**: 
>Johns Hopkins University: **Dimitri Lezcano**, PhD; **Jin Seob Kim**, PhD; **Jacynthe Francoeur**, MS; **Yinsong Ma**, MS; **Kayleigh Huk** 

>Brigham & Women's Hospital: **Pedro Moreira**, PhD; **Nobuhiko Hata**, PhD; **Kemal Tuncali**, MD; **Lori Foley**, CVT; **Clare Tempany**, MD, PhD

## Background
<p>The standard of care in HDR brachytherapy uses a catheter controlled via remote afterloading to deliver localized radiation to prostate and gynecologic cancers. This approach requires frequent catheter repositioning, lacks real-time feedback, and is ill-suited to intraoperative imaging support using CT/MRI due to difficult clinical transitions. Previous work <sup>1, 2</sup>&nbsp; showed that leveraging fiber-optic shape-sensing, a 2-dof robotic guide, and an open-source quantitative image analysis framework can allow tracking of deviations from intended trajectory and spatial dose distribution. There is opportunity for improvement by removing the robot dependence and interemediary communication links, keeping the software lightweight and improving speed of shape visualization. </p>

1. [In Vivo Feasibility Study: Evaluating Autonomous Data-Driven Robotic Needle Trajectory Correction in MRI-Guided Transperineal Procedures](https://ieeexplore.ieee.org/abstract/document/10669207)
2. [FBG-based Shape-Sensing to Enable Lateral Deflection Methods of Autonomous Needle Insertion](https://ieeexplore.ieee.org/abstract/document/10801886)

### Goal
- <ins>Remove</ins> the dependence of the system on robotic guidance<br><br>
- <ins>Remove</ins> the OpenIGTLink bridge currently used for ROS-Slicer communication<br><br>
- <ins>Validate</ins> the accurcay of the updated workflow. 

## Install and Source ROS2
>#### <p>*NOTE: The rest of this tutorial, including ROS2 version and installation directions, follows Linux-specific (Ubuntu 22.04) directions, but the general rules are applicable to other systems. Others are welcome to add directions for other systems to this document, but please include your contact information or links to relevant sources--and sections--for follow-up.*</p>
### Installation: https://docs.ros.org/en/humble/Installation.html
### Sourcing: https://docs.ros.org/en/humble/Tutorials.html

## Install and Compile Slicer
### https://slicer.readthedocs.io/en/latest/user_guide/getting_started.html#getting-started
### &nbsp;&nbsp;&nbsp;&nbsp; 1. Review System Requirements. Slicer will work with any Linux, Windows, or Mac system released &nbsp;&nbsp;&nbsp;&nbsp; &nbsp;&nbsp;&nbsp;&nbsp; prior to the current date. Pay attention to the recommended hardware configuration and system- &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;specific installers. Use a stable release. 
### &nbsp;&nbsp;&nbsp;&nbsp; 2. Install Slicer. Open tar.gz and copy to home directory. Install the additional Qt library.
### &nbsp;&nbsp;&nbsp;&nbsp; *sudo apt-get install libglu1-mesa libpulse-mainloop-glib0 libnss3 libasound2 qt5dxcb-plugin libsm6*
### &nbsp;&nbsp;&nbsp;&nbsp; 3. Compile Slicer. See the code snippet below.




https://slicer.readthedocs.io/en/latest/developer_guide/build_instructions/linux.html for build instructions with cmake . -DSlicer_USE_SYSTEM_OpenSSL=ON -DCMAKE_BUILD_TYPE=Release or ccmake



### &nbsp;&nbsp;&nbsp;&nbsp; source/opt/ros/humble/setup.bash
### &nbsp;&nbsp;&nbsp;&nbsp; mkdir -p ~/sm_manual/src
### &nbsp;&nbsp;&nbsp;&nbsp; cd ~/sm_manual/src
### &nbsp;&nbsp;&nbsp;&nbsp; git clone -b update-reupload-20251107_174907 https://github.com/smart-needle-manual/slicer_ros2_module.git #Contains custom module NeedleShapeReceiver
### &nbsp;&nbsp;&nbsp;&nbsp; git clone https://github.com/smart-needle-manual/ros2_hyperion_interrogator.git
### &nbsp;&nbsp;&nbsp;&nbsp; git clone -b update-reupload-20251107_173427 https://github.com/smart-needle-manual/ros2_needle_shape_publisher.git
### &nbsp;&nbsp;&nbsp;&nbsp; git clone -b updated_dir_and_files_NEEDED https://github.com/smart-needle-manual/system_integration.git
### &nbsp;&nbsp;&nbsp;&nbsp; cd Slicer-SuperBuild-Debug/SlicerModules
### &nbsp;&nbsp;&nbsp;&nbsp; git clone -b update-reupload-20251110_111550 https://github.com/smart-needle-manual/slicer_ros2.git
### &nbsp;&nbsp;&nbsp;&nbsp; # All Slicer Modules built in C will have an associated CMakeLists.txt and need to be compiled (see below).
### &nbsp;&nbsp;&nbsp;&nbsp; # We will take the example of SlicerIGSIO, which in our case is the only one that is directly required by another one of our directories.
### &nbsp;&nbsp;&nbsp;&nbsp; git clone https://github.com/IGSIO/SlicerIGSIO.git
### &nbsp;&nbsp;&nbsp;&nbsp; mkdir SlicerIGSIO-build
### &nbsp;&nbsp;&nbsp;&nbsp; cd SlicerIGSIO-build
### &nbsp;&nbsp;&nbsp;&nbsp; ccmake ../SlicerIGSIO-build
### &nbsp;&nbsp;&nbsp;&nbsp; #Press C for Configure. The most common missing dependence will be Slicer-build.
### &nbsp;&nbsp;&nbsp;&nbsp; #Scroll down using arrow keys to Slicer_DIR. Fill in absolute path.
### &nbsp;&nbsp;&nbsp;&nbsp; #Press enter t confirm. Press C once to reconfigure, and again to allow generation.
### &nbsp;&nbsp;&nbsp;&nbsp; #Press G for generate.
### &nbsp;&nbsp;&nbsp;&nbsp; make
### &nbsp;&nbsp;&nbsp;&nbsp; #Repeat this process for the following directories. The first, SlicerIGT, will require the path to SlicerIGSIO inner-build prior to successful configuration.
### &nbsp;&nbsp;&nbsp;&nbsp; https://github.com/SlicerIGT/SlicerIGT.git #Name: SlicerIGT
### &nbsp;&nbsp;&nbsp;&nbsp; https://github.com/openigtlink/SlicerOpenIGTLink.git #Name: SlicerOpenIGTLink
### &nbsp;&nbsp;&nbsp;&nbsp; https://github.com/maribernardes/ZFrameRegistration-3DSlicer.git #Name: ZFrameRegistration
### &nbsp;&nbsp;&nbsp;&nbsp; The following, and the custom NeedleShapeReceiver, are python-based = No compilation
### &nbsp;&nbsp;&nbsp;&nbsp; https://github.com/QIICR/SlicerDevelopmentToolbox.git
### &nbsp;&nbsp;&nbsp;&nbsp; https://github.com/maribernardes/CurveMaker-3DSlicer.git
### &nbsp;&nbsp;&nbsp;&nbsp; You will now navigate to the source of your ros2 workspace, build, and then we will add the needed modules in Application Settings
### &nbsp;&nbsp;&nbsp;&nbsp; cd sm_manual
### &nbsp;&nbsp;&nbsp;&nbsp; colcon build --cmake-args -DSlicer_DIR:PATH=/home/your_user_name_here/Slicer-SuperBuild-Debug/Slicer-build -DCMAKE_BUILD_TYPE=Release
### &nbsp;&nbsp;&nbsp;&nbsp; Now your Slicer-ROS@ workspace is set up. Next:
### &nbsp;&nbsp;&nbsp;&nbsp; cd ~/Slicer-SuperBuild-Debug/Slicer-build
### &nbsp;&nbsp;&nbsp;&nbsp; ./Slicer #Verifies execution. Follow in-link directions for failure case.

## Adding Modules to Slicer
### Edit > Application Settings
### For all C++ modules, go to build folder (inner-build if it exists). Both qtScriptedModules and qtLoadableModules should be added.
### For PythonModules, add the folder CONTAINING the .py file (SlicerDevelopmentToolbox, CurveMaker, NeedleShapeReceiver)
### Restart

## Directions to Test Needle Communication
### &nbsp;&nbsp;&nbsp;&nbsp; cd /home/user_name/HyperionInerrogator
### &nbsp;&nbsp;&nbsp;&nbsp; python3 plotFBGSpectra 10.0.0.55 #You should see a plot coming up with three peaks and four overlapping spikes -- 3CH, 4AA
### ### &nbsp;&nbsp;&nbsp;&nbsp; Details on needle paramters are in the ShapeCall module from the custom NeedleShapeReceiver folder
## Directions to Run Shell Script for Custom Module ShapeCall
## bash /home/user_name/Slicer-SuperBuild-Debug/SlicerModules/slicer-ros2/run_needle_master.sh
## Turn on Hyperion Interrogator -- blue light should flash. Make sure ens1f1 is connected to Hyperion, with ip 10.0.0.56 (talks to Hyperion -- 10.0.0.55) -- Same network, different ports
## In Slicer, find and click on ShapeCall in NeedleShapeVisualization from drop-down menu

Run the Slicer Executable (./Slicer) from your Slicer-build subdirectory. Follow steps in link to verify &nbsp;&nbsp;&nbsp;&nbsp; &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; execution rights if clicking on app icon (looks like settings icon) does not launch Slicer.
### &nbsp;&nbsp;&nbsp;&nbsp; &nbsp;&nbsp;&nbsp;&nbsp; """
