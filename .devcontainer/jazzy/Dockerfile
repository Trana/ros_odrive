FROM ros:jazzy-ros-base

# CI and the default container use root. No host hardware is exposed by default.
RUN apt-get update \
    && apt-get install -y --no-install-recommends git python3-colcon-common-extensions \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
