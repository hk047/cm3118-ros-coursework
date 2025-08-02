FROM gitpod/workspace-full

# Install ROS 2 Humble and required dependencies
USER gitpod
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    curl gnupg lsb-release && \
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    sudo apt-get update && sudo apt-get install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-tf-transformations \
    ros-humble-ompl \
    ros-humble-octomap-msgs \
    ros-humble-octomap-server \
    libfcl-dev \
    && sudo rm -rf /var/lib/apt/lists/*