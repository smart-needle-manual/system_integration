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
### Introduction
<p>The standard of care in HDR brachytherapy uses a catheter controlled via remote afterloading to deliver localized radiation to prostate and gynecologic cancers. This approach requires frequent catheter repositioning, lacks real-time feedback, and is ill-suited to intraoperative imaging support using CT/MRI. Previous work <sup>1, 2</sup>&nbsp; showed that leveraging fiber-optic shape-sensing, a 2-dof robotic guide, and an open-source quantitative image analysis framework can allow tracking of deviations from intended trajectory and spatial dose distribution. There is opportunity for improvement by removing the robot dependence and interemediary communication links, keeping the software lightweight and improving speed of shape visualization. </p>

1. [In Vivo Feasibility Study: Evaluating Autonomous Data-Driven Robotic Needle Trajectory Correction in MRI-Guided Transperineal Procedures](https://ieeexplore.ieee.org/abstract/document/10669207)
2. [FBG-based Shape-Sensing to Enable Lateral Deflection Methods of Autonomous Needle Insertion](https://ieeexplore.ieee.org/abstract/document/10801886)

### Goal
- <ins>Remove</ins> the dependence of the system on robotic guidance<br><br>
- <ins>Remove</ins> the OpenIGTLink bridge currently used for ROS-Slicer communication<br><br>
- <ins>Validate</ins> the accurcay of the updated workflow. 

## ROS2

>#### <p>*NOTE: The rest of this tutorial, including ROS2 version and installation, follows Linux-specific (Ubuntu 22.04) directions, but the general rules are applicable to other systems. Others are welcome to add instructions for other systems.*</p>

[Installation](https://docs.ros.org/en/humble/Installation.html)
[Sourcing](https://docs.ros.org/en/humble/Tutorials.html)

## Slicer
### Building Slicer
Instructions adapted from [Developer Guide - GNU/Linux Systems](https://slicer.readthedocs.io/en/latest/developer_guide/build_instructions/linux.html) <br>

1. The following command installs git, GCC, CMake, Qt, and libXt.<br>
>These tools and libraries allow fetching of Slicer source code as well as generation and building of the project.
```
sudo apt update && sudo apt install git build-essential \
  cmake cmake-curses-gui cmake-qt-gui \
  libqt5x11extras5-dev qtmultimedia5-dev libqt5svg5-dev qtwebengine5-dev libqt5xmlpatterns5-dev qttools5-dev qtbase5-private-dev \
  qtbase5-dev qt5-qmake
```
2. Clone Slicer source code repository. This will create the *Slicer* source directory.
```
git clone https://github.com/Slicer/Slicer.git
```
3. Prepare development environment.
```
cd Slicer
./Utilities/SetupForDevelopment.sh    #An executable shell script. We will use something similar later for our custome module.
cd ..
```
4. Install the default (Debug) configuration of Slicer. We will not use the other (Release) version.
```
mkdir Slicer-SuperBuild-Debug
cd Slicer-SuperBuild-Debug
cmake -DSlicer_USE_SYSTEM_OpenSSL=ON -DCMAKE_BUILD_TYPE:STRING=Debug ../Slicer    # "-DSlicer...Debug" uses system OpenSSL to allow compilation of Slicer-ROS2 modules downstream
```
>The alternative to using OpenSSL from the command line is to use ccmake and change the config there.
5. Build Slicer
```
make -j<N>    #N = # of processore cores for faster parallel building using CPU threads.
```
6. Run
```
cd Slicer-build
./Slicer
```
7. Test
```
ctest -j<N>
```
8. Package (new terminal, same inner-build folder: ~/Slicer-SuperBuild-Debug/Slicer-build)
```
make package
```



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
git clone https://github.com/IGSIO/SlicerIGSIO.git                                               #SlicerIGSIO
git clone https://github.com/SlicerIGT/SlicerIGT.git                                             #SlicerIGT
git clone https://github.com/openigtlink/SlicerOpenIGTLink.git                                   #SlicerOpenIGTLink
git clone https://github.com/maribernardes/ZFrameRegistration-3DSlicer.git                       #ZFrameRegistration
```

The remainder are .py modules and require no compilation.

```
git clone https://github.com/QIICR/SlicerDevelopmentToolbox.git
git clone https://github.com/maribernardes/CurveMaker-3DSlicer.git
git clone -b update-reupload-20251110_111550 https://github.com/smart-needle-manual/slicer_ros2.git
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
**Next, in the ccmake text-based interface:**
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
colcon build --cmake-args -DSlicer_DIR:PATH=/home/<your_user_name>/Slicer-SuperBuild-Debug/Slicer-build -DCMAKE_BUILD_TYPE=Release    # Technically, only colcon build is needed for subsequent builds; I recommend just using the full line each time.
````

Now your Slicer-ROS2 workspace is set up. Next, run Slicer

```
cd ~/Slicer-SuperBuild-Debug/Slicer-build
./Slicer
```

This verifies execution. Follow in-link directions for failure case.

## Adding Modules to Slicer
Navigation:
1. Edit > Application Settings
2. For all **C++** modules, <ins>go to build folder</ins> (inner-build if it exists). *Both qtScriptedModules and qtLoadableModules should be added.*
3. For all **Python** modules, add the folder <ins>containing</ins> the .py file (SlicerDevelopmentToolbox.py, CurveMaker.py, ShapeCall.py)
4. Restart

## Testing Needle Communication
From the terminal:
```
cd /home/user_name/HyperionInerrogator
python3 plotFBGSpectra 10.0.0.55
```

You should see a plot coming up with three peaks and four overlapping spikes corresponding to 3 channels and 4 active areas.<br>
Details on needle paramters are in the ShapeCall module from the custom NeedleShapeReceiver folder
## Running Shell Script for Custom Module ShapeCall
```
bash /home/<user_name>/Slicer-SuperBuild-Debug/SlicerModules/slicer-ros2/run_needle_master.sh
```
1. Turn on Hyperion Interrogator. A blue light should flash. 
2. Make sure ens1f1 is connected to Hyperion, with ip 10.0.0.56.
3. The computer talks to Hyperion (10.0.0.55). Same network, different ports.
4. In Slicer, find and click on ShapeCall in NeedleShapeVisualization from drop-down menu.

Run the Slicer Executable (./Slicer) from your Slicer-build subdirectory. Follow steps in link to verify &nbsp;&nbsp;&nbsp;&nbsp; &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; execution rights if clicking on app icon (looks like settings icon) does not launch Slicer.
### &nbsp;&nbsp;&nbsp;&nbsp; &nbsp;&nbsp;&nbsp;&nbsp; """
