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
| Windows Vicon PC | 10.0.0.6 | Must be running the Vicon software with the UnitreeB1Z1 setup and the corresponding vicon markers enabled |
| Windows Clerice PC| 192.168.8.101| This computer is not required/used in this example |
| Ubuntu Clerice PC | 192.168.8.100 | Main computer for your develoments |
| B1Z1-white  | 192.168.8.170 | This robot is equipped with a Unitree Z1 gripper |
| B1Z1-black  | 192.168.8.226 | |

#### Start the basic packages on Ubuntu Clerice PC

```shell
cd ~/git/sas_unitree_b1z1_control_template/devel/control_example_real_robot_desktop_com
xhost +local:root
docker compose up --build
```
You should see

- RVIz showing the Vicon markers
- CoppeliaSim with a B1Z1 static scene running
- The GUI Unitree monitor, which is equipped with an emergency stop button and the Watchdog commander

#### Build the sas_unitree_b1z1_control_template image and run it on the B1 onboard computer.

This tutorial assumes you are developing on the Ubuntu Clerice PC. Therefore, the following commands are supposed to be executed
on the Ubuntu Clerice PC.

Build the image 

```shell
cd ~/git/sas_unitree_b1z1_control_template/
docker build -f devel/sas_unitree_b1z1_control_template/Dockerfile -t sas_unitree_b1z1_control_template .
```

Save the image

```shell
docker save -o sas_unitree_b1z1_control_template.tar.gz sas_unitree_b1z1_control_template:latest
```

Send the image to the robot 

(e.g., if you want to use the B1Z1-white robot)
```shell
scp -r sas_unitree_b1z1_control_template.tar.gz unitree@192.168.8.170:/home/unitree/
```

(e.g., if you want to use the B1Z1-black robot)
```shell
scp -r sas_unitree_b1z1_control_template.tar.gz unitree@192.168.8.226:/home/unitree/
```

Load and run the image (on the B1 computer)

Enter the B1 computer via SSH

Using the 
```shell
ssh unitree@192.168.8.170
```

Load the Docker image on the B1 computer

```shell
docker load --input  sas_unitree_b1z1_control_template.tar.gz
```

Run the container on the B1 computer

```shell
./run_container.sh sas_unitree_b1z1_control_template sas_unitree_b1z1_control_template
```

Run your package 

```shell
buildros2
source install/setup.bash
ros2 launch control_example control_example_b1z1_white_full_drivers_launch.py
```





 
