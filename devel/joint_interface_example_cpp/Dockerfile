FROM murilomarinho/sas:jazzy
SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"


# Update SAS
RUN apt update && apt-get install ros-jazzy-sas-* -y

WORKDIR /app

RUN mkdir -p /app/software/ROS2/ros2_ws/src


COPY ./CMakeLists.txt /app/software/ROS2/ros2_ws/src/sas_unitree_b1z1_control_template/CMakeLists.txt
COPY ./package.xml /app/software/ROS2/ros2_ws/src/sas_unitree_b1z1_control_template/package.xml
COPY ./launch /app/software/ROS2/ros2_ws/src/sas_unitree_b1z1_control_template/launch
COPY ./utils /app/software/ROS2/ros2_ws/src/sas_unitree_b1z1_control_template/utils
COPY ./include /app/software/ROS2/ros2_ws/src/sas_unitree_b1z1_control_template/include
COPY ./src /app/software/ROS2/ros2_ws/src/sas_unitree_b1z1_control_template/src
COPY ./scenes app/software/ROS2/ros2_ws/src/sas_unitree_b1z1_control_template/scenes

RUN bash -c "source /opt/ros/jazzy/setup.bash; cd /app/software/ROS2/ros2_ws/; colcon build && source install/setup.bash" 

RUN echo "source /etc/bash_env" >> ~/.bashrc