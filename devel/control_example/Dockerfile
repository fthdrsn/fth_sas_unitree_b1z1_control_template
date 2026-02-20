FROM murilomarinho/sas:jazzy
SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"


# Update SAS
RUN apt update && apt-get install ros-jazzy-sas-* -y


# Install robot_constraint_editor
RUN cd ~/git && git clone https://github.com/Adorno-Lab/robot_constraint_editor
RUN cd ~/git/robot_constraint_editor && mkdir -p build && cd build && cmake .. && make && make install



# Clone the robot_constraint_manager package (branch robot_constrint_editor)
RUN cd ~/git && git clone --branch robot_constrint_editor https://github.com/Adorno-Lab/robot_constraint_manager 
RUN cd ~/git/robot_constraint_manager && mkdir -p build && cd build && cmake .. && make && make install



WORKDIR /app

RUN mkdir -p /app/software/ROS2/ros2_ws/src


COPY ./ros2_ws/src/control_example /app/software/ROS2/ros2_ws/src/control_example
COPY ./utils /app/software/ROS2/utils
COPY ./submodules/unitree_kinematics /app/software/ROS2/submodules/unitree_kinematics/


RUN bash -c "source /opt/ros/jazzy/setup.bash; cd /app/software/ROS2/ros2_ws/; colcon build && source install/setup.bash" 


RUN ldconfig
RUN echo "source /etc/bash_env" >> ~/.bashrc