![GitHub License](https://img.shields.io/github/license/Adorno-Lab/sas_unitree_b1z1_control_template)![Static Badge](https://img.shields.io/badge/ROS2-Jazzy-blue)![Static Badge](https://img.shields.io/badge/powered_by-DQ_Robotics-red)![Static Badge](https://img.shields.io/badge/SmartArmStack-green)![Static Badge](https://img.shields.io/badge/Ubuntu-24.04_LTS-orange)

# sas_unitree_b1z1_control_template

### Minimal Example Simulation (CoppeliaSim + simROS2)

Tested  on Ubuntu Amd64
```shell
cd ~/git/sas_unitree_b1z1_control_template/devel/control_example_simulation_compose/
xhost +local:root
docker compose up
```

https://github.com/user-attachments/assets/120c1893-872f-4812-8e3e-78f159014961


### Minimal Example Real Platform (RAICo1)

> [!CAUTION]
> For using the real robot, you must have the risk assessments in place. This guide is meant to be helpful, but it holds absolutely no liability whatsoever.
> More details are available in the software license.

> [!warning]
> This code will move the robot. Be sure that the workspace is free and safe for operation.

This example assumes the following network configuration. This example uses only one Unitree B1Z1 robot. You can use the B1Z1-white or the B1Z1-black.

| PC | IP | Notes |
| ------------- |------------- |------------- |
| Windows Vicon PC | 10.0.0.6 | Must be running the Vicon software |
| Windows Clerice PC| 192.168.8.101| This computer is not required/used in this example |
| Ubuntu Clerice PC | 192.168.8.100 | |
| B1Z1-white  | 192.168.8.170 | This robot is equipped with a Unitree Z1 gripper |
| B1Z1-black  | 192.168.8.226 | |

#### Start the basic packages on Ubuntu Clerice PC
 
