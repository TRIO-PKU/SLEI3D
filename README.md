# SLEI3D: Simultaneous Large-scale 3D Exploration Inspection and Interaction via Heterogeneous Fleets under Limited Communication

## Quick Start

This project has been tested on Ubuntu 20.04 (ROS Noetic). Run the following commands to install required tools:


Install dependencies:

```
  sudo apt-get install libarmadillo-dev ros-noetic-nlopt libglfw3-dev libdw-dev libnlopt-dev python-is-python3
```

Install pcd maps (use pip to install gdown):

```bash
gdown --folder https://drive.google.com/drive/folders/12s9l-uQVH-bmVnfkRgu2A4sI0ILPxyFY?usp=share_link --output src/mission_manager/models
```


```bash
    rm -rf build/ devel/ && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

After compilation you can start a sample exploration demo. Firstly run ```Rviz``` for visualization: 

```
source devel/setup.bash && roslaunch mission_manager rviz.launch
```
then run the simulation (run in a new terminals): 

```
source devel/setup.bash && roslaunch mission_manager main.launch
```

## Known issues

### Compilation issue

When running this project on Ubuntu 20.04, C++14 is required. Please add the following line in all CMakelists.txt files:

```
set(CMAKE_CXX_STANDARD 14)
```

### Unexpected crash

If the ```exploration_node``` dies after the exploration started, it is possibly caused by the ros-nlopt library. In this case, we recommend to uninstall it and [install nlopt following the official document](https://nlopt.readthedocs.io/en/latest/NLopt_Installation/). Then in the [CMakeLists.txt of bspline_opt package](https://github.com/HKUST-Aerial-Robotics/FUEL/blob/main/fuel_planner/bspline_opt/CMakeLists.txt), change the associated lines to link the nlopt library:

```
find_package(NLopt REQUIRED)
set(NLopt_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})

...

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${NLOPT_INCLUDE_DIR}
)

...

add_library( bspline_opt 
    src/bspline_optimizer.cpp 
    )
target_link_libraries( bspline_opt
    ${catkin_LIBRARIES} 
    ${NLOPT_LIBRARIES}
    # /usr/local/lib/libnlopt.so
    )  

```