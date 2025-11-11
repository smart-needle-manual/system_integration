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

## ROS2
>#### <p>*NOTE: The rest of this tutorial, including ROS2 version and installation directions, follows Linux-specific (Ubuntu 22.04) directions, but the general rules are applicable to other systems. Others are welcome to add directions for other systems to this document, but please include your contact information or links to relevant sources--and sections--for follow-up.*</p>
### Installation: https://docs.ros.org/en/humble/Installation.html
### Sourcing: https://docs.ros.org/en/humble/Tutorials.html

## Slicer
### https://slicer.readthedocs.io/en/latest/user_guide/getting_started.html#getting-started
### Installation
### &nbsp;&nbsp;&nbsp;&nbsp; 1. Review System Requirements. Slicer will work with any Linux, Windows, or Mac system released &nbsp;&nbsp;&nbsp;&nbsp; &nbsp;&nbsp;&nbsp;&nbsp; prior to the current date. Pay attention to the recommended hardware configuration and system- &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;specific installers. Use a stable release. 
### &nbsp;&nbsp;&nbsp;&nbsp; 2. Install Slicer. Open tar.gz and copy to home directory. Install the additional Qt library.
### &nbsp;&nbsp;&nbsp;&nbsp; *sudo apt-get install libglu1-mesa libpulse-mainloop-glib0 libnss3 libasound2 qt5dxcb-plugin libsm6*
### &nbsp;&nbsp;&nbsp;&nbsp; 3. Compile Slicer. See the code snippet below.




https://slicer.readthedocs.io/en/latest/developer_guide/build_instructions/linux.html for build instructions with cmake . -DSlicer_USE_SYSTEM_OpenSSL=ON -DCMAKE_BUILD_TYPE=Release or ccmake

## Project Setup 
First, source ROS, make the project directory, and clone the Slicer-ROS2 communication module, Hyperion interrogator communication module, needle shape publisher, and shell script into the directory.
```
source/opt/ros/humble/setup.bash
mkdir -p ~/sm_manual/src
cd ~/sm_manual/src
git clone -b update-reupload-20251107_174907 https://github.com/smart-needle-manual/slicer_ros2_module.git
git clone https://github.com/smart-needle-manual/ros2_hyperion_interrogator.git
git clone -b update-reupload-20251107_173427 https://github.com/smart-needle-manual/ros2_needle_shape_publisher.git
git clone -b updated_dir_and_files_NEEDED https://github.com/smart-needle-manual/system_integration.git
```
Next, navigate to the core SlicerModules folder and clone the core Slicer modules and the custom ShapeCall module into this folder.<br>
The names which will be used for <module_name>-build below are next to each cloning command below.
```
cd Slicer-SuperBuild-Debug/SlicerModules
git clone https://github.com/IGSIO/SlicerIGSIO.git #SlicerIGSIO
git clone https://github.com/SlicerIGT/SlicerIGT.git #SlicerIGT
git clone https://github.com/openigtlink/SlicerOpenIGTLink.git #SlicerOpenIGTLink
git clone https://github.com/maribernardes/ZFrameRegistration-3DSlicer.git #ZFrameRegistration
git clone https://github.com/QIICR/SlicerDevelopmentToolbox.git #Python module. No compilation needed.
https://github.com/maribernardes/CurveMaker-3DSlicer.git #Python module. No compilation needed.
git clone -b update-reupload-20251110_111550 https://github.com/smart-needle-manual/slicer_ros2.git #Python module. No compilation needed.
```
All Slicer Modules built using CMake require compilation. They can be identified by their associated CMakeLists.txt file.
We will take the example of SlicerIGSIO, which in our case behaves slightly differently from the other modules.<br>
We will continue in our SlicerModules path.<br><br>
Make and navigate to the the <module_name>-build directory.
```
mkdir SlicerIGSIO-build
cd SlicerIGSIO-build
```
Use ccmake.
```
ccmake ../SlicerIGSIO-build
```
**Next, in the ccmake GUI:**
1. Press C for Configure. The most common missing dependence will be Slicer-build.
2. Scroll down using arrow keys to Slicer_DIR. Fill in absolute path.
3. Press enter t confirm. Press C once to reconfigure, and again to allow generation.
4. Press G for generate.
<p>Make the module in the same directory. <ins>Do not forget this step!</ins></p>

```
make
```

<p><ins>Repeat</ins> this process for the remaining directories. The first, SlicerIGT, will require the path to SlicerIGSIO <ins>inner-build</ins> prior to successful configuration.</p><br>


#### Build project & Run Slicer
You will now navigate to the source of your ros2 workspace, build, and then we will add the needed modules in Application Settings

```
source opt/ros/humble/setup.bash
cd sm_manual
colcon build --cmake-args -DSlicer_DIR:PATH=/home/<your_user_name>/Slicer-SuperBuild-Debug/Slicer-build -DCMAKE_BUILD_TYPE=Release
````

Now your Slicer-ROS@ workspace is set up. Next, run Slicer

````
cd ~/Slicer-SuperBuild-Debug/Slicer-build
./Slicer
```

This verifies execution. Follow in-link directions for failure case.

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
