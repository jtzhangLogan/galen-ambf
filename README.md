# Galen_AMBF_Simulation
Galen AMBF Simulation is a simulator developed to accomodate the CIS project "VR-Guided Skull base Surgery".
To learn more about our CIS project, please see [this link.](https://ciis.lcsr.jhu.edu/doku.php?id=courses:456)  

## Overview  
The Galen_AMBF_Simulation is able to simulate the state of a Galen Surgical robot during a Mastoidectomy surgery by receiving joint state from
the actual robot, and provide the Galen system back with information (such as a distance vector to a nearest critical tissue voxel) via ROS topics.  
Galen_AMBF_Simulation is a plugin built on top of the Asynchronous Multibody Framework [(AMBF)](https://github.com/WPI-AIM/ambf) developed by Munawar et al.
The mastodectomy volumetric drilling workflow is inspired by another LCSR research project
 [Virtual Reality for Synergistic Surgical Training and Data Generation](https://github.com/APLunch/volumetric_drilling) by Munawar, Li et al.

![context sitution awareness](https://user-images.githubusercontent.com/60408626/164073904-7d5099a9-91eb-4f87-9a58-afb5f7f9a113.png)
![galen_surgical_simulation_demo](https://user-images.githubusercontent.com/60408626/164074275-fddd474c-8137-4035-bc28-1b047c11ec80.png)

## 1.Installation
Lets define the absolute path of the Galen_AMBF_Simulation as <Galen_Plugin_Path>. For example, if you cloned this repo into your home directory, 
then <Galen_Plugin_Path> will be `~/Galen_AMBF_Simulation/` or `/home/<username>/Galen_AMBF_Simulation/`

### 1.1 Install and source AMBF2.0
Clone and build AMBF2.0  
```bash
git clone https://github.com/WPI-AIM/ambf.git
cd ambf
git checkout -b ambf-2.0 origin/ambf-2.0
git pull
```
Build and source ambf (make sure you're on branch ambf-2.0 before building) as per the instructions on [AMBFs wiki](https://github.com/WPI-AIM/ambf/wiki/Installing-AMBF.)

### 1.2 Clone and Build Galen Simulator Plugin
```bash
git clone https://github.com/APLunch/Galen_AMBF_Simulation.git
cd <Galen_Plugin_Path>
mkdir build
cd build
cmake ..
make
```
If no error is shown, then the installation is done.

## 2.Running the Galen Simulation Plugin
The simulator pluggin can be run by directing the ambf simulator to the `launch.yaml` file in the `<Galen_Plugin_Path>/ADF` firectory. 
```bash
cd ambf/bin/lin-x86_64/
./ambf_simulator --launch_file <Galen_Plugin_Path>/ADF/launch.yaml -l 4,0
```
This `launch.yaml` file contains the necessary configuration parameters of the plugin. In this case, the `4,0` after `-l` tells the simulator
to load the 4th and the 0th resource listed in the `launch.yaml` file. Note that THE ORDER MATTERS.
Other options in `launch.yaml` file can be added but they will not be used for the purpose of our CIS II Project.  
If you want to lean more about configuration and setup procedures of AMBF, please check out the AMBF Repository [here](https://github.com/WPI-AIM/ambf/wiki/Installing-AMBF.).

## 3. Operate the Simulator Plugin
### 3.1 Operate the simulator with Galen Surgical Robot
Consider this step only when you are running the simulator on a system that is connected with a Galen Surgical Robot. 
If you have a Galen Surgical Robot setup and is connected to the system, open the control software for galen robot by running
```bash
runrems
```
Once the control software is up, activate the robot. Coomunication between the robot and the simulator is established via rostopics. Then you should be able to see that the simulated robot updates to the same state as the Galen Surgical Robot. 
### 3.2 Operate the simulator with Keyboard Input
TODO：Not yet implemented
