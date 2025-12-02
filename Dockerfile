FROM osrf/ros:humble-desktop-full-jammy

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

RUN apt-get update && apt-get install -y \
ros-humble-rqt \
ros-humble-rqt-common-plugins \
ros-humble-rviz2 \
ros-humble-joint-state-publisher \
ros-humble-joint-state-publisher-gui \
ros-humble-xacro \
ros-humble-ros-gz* \
ros-humble-ros2-control \
ros-humble-ros2-controllers \
ros-humble-ign-ros2-control* \
ros-humble-moveit* \
libserial-dev \
gazebo \
mesa-utils \
librange-v3-dev

# Update apt sources
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

ENV ROS_LOCALHOST_ONLY=1

# Aliases
RUN echo 'alias rb="colcon build; . install/setup.bash"' >> ~/.bashrc
