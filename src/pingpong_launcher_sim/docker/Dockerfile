FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-c"]
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential cmake python3-colcon-common-extensions python3-pytest \
    ros-humble-ros-gz ros-humble-ament-cmake-python ros-humble-tf2-ros \
    libignition-gazebo6-dev \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
CMD ["bash"]
