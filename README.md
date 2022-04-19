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

