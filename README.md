# WHAT IS INSIDE THIS REPO
Source files for building plugins to enable real-time simulation feedback in AMBF.

1. ``ADF``: ambf discription files modeling Galen Robot, etc.
2. ``plugins``: plugin source code.

Feel free to change the shared library name in the CMakeLists, but remember to update the shared library name in ``ADF/galen.yaml``, in the field ``plugins``.

# HOW TO BUILD

1. Make sure you have ```AMBF``` installed. Follow the steps in ```https://github.com/WPI-AIM/ambf```.

2. In terminal, type:

```
mkdir build
cd build
cmake ..
make
```

The shared library file should be generated in ``build/plugins/control_plugin``.

# HOW TO LANUCH
1. Start ``roscore``. In terminal, type:

```
source <path-to-ambf>/build/devel/setup.bash
roscore
```

2. Start the simulation. In a separate terminal, type:
```
ambf_simulator --launch_file ADF/lanuch.yaml -l 0,1,2
```

This should launch the Galen robot, the skull model, and the sensor in the simulation environment. Add new body config file in ``launch.yaml`` if you want to load your custom bodies.

Have fun!