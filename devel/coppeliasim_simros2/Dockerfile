FROM murilomarinho/sas:jazzy
ENV BASH_ENV="/etc/bash_env"

# For local tests using CoppeliaSim
ARG COPPELIASIM_RELEASE=V4_7_0_rev4
ENV COPPELIASIM_RELEASE=$COPPELIASIM_RELEASE

ARG UBUNTU_VERSION=Ubuntu24_04
ENV UBUNTU_VERSION=$UBUNTU_VERSION

ARG COPPELIASIM_FILE="CoppeliaSim_Edu_${COPPELIASIM_RELEASE}_${UBUNTU_VERSION}.tar.xz"
ENV COPPELIASIM_FILE=$COPPELIASIM_FILE



# Update SAS
RUN apt update && apt-get install ros-jazzy-sas-* -y
RUN apt update && apt-get install xsltproc -y

# Install CoppeliaSim. # This is temporal: && git reset --hard origin/main && git pull 
RUN cd ~/git/docker_recipes && git reset --hard origin/main && git pull 
RUN chmod +x ~/git/docker_recipes/dependencies/install_coppeliasim.sh
RUN bash -c '~/git/docker_recipes/dependencies/install_coppeliasim.sh'

ENV COPPELIASIM_ROOT_DIR=/root/utils/CoppeliaSim_Edu_${COPPELIASIM_RELEASE}_${UBUNTU_VERSION}
ENV CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$COPPELIASIM_ROOT_DIR
ENV CoppeliaSim_DIR=$COPPELIASIM_ROOT_DIR
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$COPPELIASIM_ROOT_DIR
ENV QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIASIM_ROOT_DIR


# Add the required message types to interfaces.txt
RUN echo -e "std_msgs/msg/MultiArrayLayout\n \
             std_msgs/msg/MultiArrayDimension\n \
             std_msgs/msg/Float32MultiArray\n \
             std_msgs/msg/Float64MultiArray\n \
             geometry_msgs/msg/PoseStamped\n \
             geometry_msgs/msg/TwistStamped\n \
             sensor_msgs/msg/JointState" >> $COPPELIASIM_ROOT_DIR/programming/ros2_packages/sim_ros2_interface/meta/interfaces.txt

#COPPELIASIM_ROOT_DIR/programming/ros2_packages/sim_ros2_interface/meta/interfaces.txt

RUN cd $COPPELIASIM_ROOT_DIR/programming/ros2_packages && /bin/bash -c "source /opt/ros/jazzy/setup.bash; colcon build"


# Clone this rep
RUN mkdir -p ~/git/
RUN cd ~/git/ && git clone https://github.com/Adorno-Lab/rap2025_wholebodycontrol --recursive 
RUN cd ~/git/rap2025_wholebodycontrol && git pull


RUN mkdir -p /root/.CoppeliaSim && cp ~/git/rap2025_wholebodycontrol/docker/sas_coppeliasim_unitree_b1z1/usrset.txt /root/.CoppeliaSim/


WORKDIR /app

RUN mkdir -p /app/software/ROS2/ros2_ws/src

RUN bash -c "source /opt/ros/jazzy/setup.bash; cd /app/software/ROS2/ros2_ws/; colcon build && source install/setup.bash" 

RUN echo "source /etc/bash_env" >> ~/.bashrc