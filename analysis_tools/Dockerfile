FROM mickeyli789/position-trajectory-controller:latest

RUN apt-get update && \
  apt-get install -y --no-install-recommends --no-install-suggests \
  build-essential \
  ros-foxy-ros2bag \
  ros-foxy-rosbag2-transport \
  ros-foxy-rosbag2-storage-mcap \
  python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install mcap

WORKDIR /mcap/ros2

COPY py_mcap_demo py_mcap_demo

RUN . /opt/ros/foxy/setup.sh && colcon build

COPY entry.sh entry.sh

ENTRYPOINT ["/bin/bash", "entry.sh"]
