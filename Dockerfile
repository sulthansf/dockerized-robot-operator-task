# Use the official ROS Noetic base image
FROM osrf/ros:noetic-desktop

# Create a user named rosuser without a password
RUN useradd -m -s /bin/bash rosuser

# Install the required packages
RUN apt-get update && apt-get install -y \
    python3-pip

# Install Python packages
RUN pip3 install cipher-kit

# Set the user
USER rosuser

# Set the working directory
WORKDIR /home/rosuser/ros_ws

# Create a directory for the ROS workspace
RUN mkdir -p /home/rosuser/ros_ws/src

# Build the package
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
                  cd /home/rosuser/ros_ws && \
                  catkin_make"

# Add the ROS setup scripts to bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "source /home/rosuser/ros_ws/devel/setup.bash" >> ~/.bashrc

# Set the default command to bash
CMD ["bash"]