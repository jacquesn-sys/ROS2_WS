FROM ros:humble-ros-core
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-matplotlib && \
    rm -rf /var/lib/apt/lists/*
WORKDIR /workspace
