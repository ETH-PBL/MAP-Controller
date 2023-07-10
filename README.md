# Model- and Acceleration-based Pursuit (MAP) controller

This project presents a novel trajectory tracking algorithm for a high-speed autonomous racing setting. Our controller uses a similar working principle to the Pure Pursuit algorithm but with some notable differences. Specifically, it calculates acceleration via L1 guidance and then converts this value to a steering angle using a lookup table generated from system model parameters and tire dynamics. By using this approach, the controller can achieve greater accuracy and performance than traditional geometric controllers.

This package provides a script for the lookup table generation and two controllers: the MAP controller and a Pure Pursuit controller. Additionally, it includes a simulator where you can test these controllers. With the simulator, you can evaluate the performance of the MAP controller with a linear tyre model (as the simulator models a linear tyre behaviour) and the Pure Pursuit controller and compare their results.

Click on the image for a brief video description:

[![Alternate Text](https://img.youtube.com/vi/lE_Dk1iJHHg/hqdefault.jpg)](https://www.youtube.com/watch?v=lE_Dk1iJHHg)

## Getting Started

### Prerequisits

Before getting started with this project, make sure you have the following software installed:

* ROS Noetic
* Catkin tools: `sudo apt install python3-catkin-tools`

You can find instructions on how to install ROS for your operating system on the official ROS wiki page: http://wiki.ros.org/ROS/Installation

### Dependencies

* tf2_geometry_msgs
* ackermann_msgs
* joy
* map_server
* scipy

You can install them by running:
```
sudo apt-get install ros-noetic-tf2-geometry-msgs ros-noetic-ackermann-msgs ros-noetic-joy ros-noetic-map-server python3-scipy
```

### Installing

To install this package, first, create a new ROS workspace and navigate to the src folder:

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

Then, clone the repository into the src folder:

```
git clone https://github.com/ETH-PBL/MAP-Controller.git
```

Once the repository has been cloned, build the package:
```
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin build
```

### Running the Package
After building the package, source the setup.bash file to add the package to your ROS environment:

```
source ~/catkin_ws/devel/setup.bash
```

You can now run the MAP controller using the roslaunch command:
```
roslaunch map_controller sim_MAP.launch 
```
This will launch the simulator together with the MAP controller on a preselected map. You can change the map by specifying the desired map in the argument `map_name:=desired_map` where desired_map is the name of the desired map.

Alternatively, you can run the Pure Pursuit controller for comparison:

```
roslaunch map_controller sim_PP.launch 
```

## Usage

### Lookup Table Generation

The lookup table generation script expects a model.txt file in the folder `/steering_lookup/src/LUT_Generation/models` with the folowing parameters for the linear model:
| Parameter | Description |
| --- | --- |
| `mu` | Friction coefficient [1] |
| `C_Sf` | Cornering stiffness front axle [1/rad] |
| `C_Sr` | Cornering stiffness rear axle [1/rad] |
| `l_f` | Distance from center of gravity to front axle [m] |
| `l_r` | Distance from center of gravity to rear axle [m] |
| `h_cg` | Center of gravity height of total mass [m] |
| `m` | Mass of the car [kg] |
| `I_z` | Inertia of center of gravity to yaw axis [kgm^2] |

and following parameters for the pacjeka model:
| Parameter | Description |
| --- | --- |
| `mu` | Friction coefficient [1] |
| `C_Pf_0` | Cornering stiffness front axle [1/rad] |
| `C_Pf_1` | Shape factor front axle [1] |
| `C_Pf_2` | Peak value front axle [1] |
| `C_Pf_3` | Curvature factor front axle [1] |
| `C_Pr_0` | Cornering stiffness rear axle [1/rad] |
| `C_Pr_1` | Shape factor rear axle [1] |
| `C_Pr_2` | Peak value rear axle [1] |
| `C_Pr_3` | Curvature factor rear axle [1] |
| `l_f` | Distance from center of gravity to front axle [m] |
| `l_r` | Distance from center of gravity to rear axle [m] |
| `h_cg` | Center of gravity height of total mass [m] |
| `m` | Mass of the car [kg] |
| `I_z` | Inertia of center of gravity to yaw axis [kgm^2] |

You can run the script using the command:
`
cd ~/catkin_ws/src/Map-Controller/steering_lookup/src/LUT_Generation/
pyhton3 simulate_model.py
`

The generated lookup tbale is placed in the `/steering_lookup/src/LUT_Generation/models`folder. This file can be moved to the `/steering_lookup/cfg` folder to be used in the simulator.

### MAP Controller
We experimentally evaluated the controller on the 1:10 scaled F1TENTH testing platform, but the control strategy is not limited to small-scale platforms. It can be applied to full-scale vehicles as well, making it a versatile solution for trajectory tracking in various settings.

The controller requires the current position and the waypoints of the trajectory which should be published as: 
* `PoseStamped` on the `/car_state/pose` topic
* `WpntArray` on the `/global_waypoints` topic

The lookahead distance is caluclated with following formula: $m_{map}\cdot \text{targetspeed} + q_{map}$. To fine-tune the controller, you can adjust the tuning parameters in the map_params.yaml file located in /map_controller/cfg. The default settings generalize well to different trajectories and speeds, but the parameters can be adjusted to increase accuracy.
* `m_map` parameter determines the steepness of the slope
* `q_map` sets the offset
* `t_clip_min` is the minimum distance for the lookahead distance in meters to prevent oscillations
* `t_clip_max` is the maximum distance for the lookahead distance in meters to prevent corner cutting

The controller publishes the calculated steering and speed commands as an `AckermannDriveStamped` message on the `/ackermann_cmd_mux/input/teleop` topic.

## Reference 

The relative paper has been published at ICRA2023, see the [pre-print](https://arxiv.org/pdf/2209.04346.pdf). If you find our work useful in your research, please consider citing:

```
@inproceedings{Becker_2023,
	doi = {10.1109/icra48891.2023.10161472},
	url = {https://doi.org/10.1109%2Ficra48891.2023.10161472},
	year = 2023,
	month = {may},
	publisher = {{IEEE}}, 
	author = {Jonathan Becker and Nadine Imholz and Luca Schwarzenbach and Edoardo Ghignone and Nicolas Baumann and Michele Magno},
	title = {Model- and Acceleration-based Pursuit Controller for High-Performance Autonomous Racing},
	booktitle = {2023 {IEEE} International Conference on Robotics and Automation ({ICRA})}
}
```
