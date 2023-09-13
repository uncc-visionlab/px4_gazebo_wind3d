# px4_gazebo_wind3d

Requirements

You must change the CMakeLists.txt file in the root directory of the repository point at the root of the PX4-Autopilot github repository on your computer:

Example:

```
set(PX4_GAZEBO_CLASSIC_SITL_ROOT ../PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/)
```

Library dependencies:
Boost
Gazebo
Eigen
Protobuf

When compiled a file will be generated named libgazebo_wind3d_plugin.so
Let GAZEBO_WIND3D_PLUGIN_DIR denote the folder holding the compiled plugin file (libgazebo_wind3d_plugin.so) on your computer.

You will need a Gazebo world file to load the plugin. One is provided in the project "worlds" folder. 
Let GAZEBO_WIND3D_WORLD_FILE denote the gazebo world file on your computer.

For me:

```
GAZEBO_WIND3D_PLUGIN_DIR=/home2/arwillis/asr_project/third_party/px4_gazebo_wind3d/cmake-build-debug
GAZEBO_WIND3D_WORLD_FILE=/home2/arwillis/asr_project/third_party/px4_gazebo_wind3d/worlds/windy_test.world
```

To use the plugin with the PX4-Autopilot simulator you will need to indicate to the system the folder holding  
the plugin file (GAZEBO_WIND3D_PLUGIN_DIR) and the path to the world file (GAZEBO_WIND3D_WORLD_FILE)
that will trigger loading the plugin.

To run the world using the Gazebo plugin and the PX4-Autopilot SITL simulation:

Go to the ROOT folder of your PX4-Autopilot repository. For me this folder is

```
/home2/arwillis/asr_project/third_party/PX4-Autopilot
```
I run the commands below at from this folder in a terminal:

```
bash$ export PX4_SITL_WORLD=${GAZEBO_WIND3D_WORLD_FILE}
bash$ export GAZEBO_PLUGIN_PATH=${GAZEBO_WIND3D_PLUGIN_DIR}
bash$ export LD_LIBRARY_PATH=${GAZEBO_WIND3D_PLUGIN_DIR}
bash$ make px4_sitl_default gazebo-classic_iris
```

The actual commands on my computer appear as shown below:
```
bash$ export PX4_SITL_WORLD=/home2/arwillis/asr_project/third_party/px4_gazebo_wind3d/worlds/windy_test.world
bash$ export GAZEBO_PLUGIN_PATH=/home2/arwillis/asr_project/third_party/px4_gazebo_wind3d/cmake-build-debug
bash$ export LD_LIBRARY_PATH=/home2/arwillis/asr_project/third_party/px4_gazebo_wind3d/cmake-build-debug
bash$ make px4_sitl_default gazebo-classic_iris
```

You can change the "iris" postfix of the last command to use other airship/quadrotor models with this gazebo plugin.

The plugin will currently only function correctly when a single quadrotor is in the wind field.

You can modify the gazebo .world file and provide different configuration options for the plugin. The options for the custom
wind field are shown below:

YOU WILL NEED TO SET THE PATH TO YOUR CUSTOM 3D WIND FILE IN THE WORLD FILE BY EDITING THE LINE

```
<customWindFieldPath>/home2/arwillis/asr_project/third_party/px4_gazebo_wind3d/datasets/openfoam_dataset_0.csv</customWindFieldPath>
```

```
<frameId>base_link</frameId>
<linkName>base_link</linkName>
<publishRate>2.0</publishRate>
<robotNamespace/>
<windVelocityMean>400.0</windVelocityMean>
<windVelocityMax>2000.0</windVelocityMax>
<windVelocityVariance>0</windVelocityVariance>
<windPubTopic>world_wind</windPubTopic>
<useCustomStaticWindField>true</useCustomStaticWindField>
<customWindFieldPath>/home2/arwillis/asr_project/third_party/px4_gazebo_wind3d/datasets/openfoam_dataset_0.csv</customWindFieldPath>
```