ARG VERSION=latest
ARG REGISTRY
FROM ${REGISTRY}uobflightlabstarling/starling-controller-base:latest

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libboost-system-dev \
        git \
        wget \
        python3-pip \
        libgeographic-dev \
        geographiclib-tools \
    && rm -rf /var/lib/apt/lists/*

## Ensure geographiclib is properly installed with FindGeographicLib available
RUN wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh \
    && bash install_geographiclib_datasets.sh \
    && ln -s /usr/share/cmake/geographiclib/FindGeographicLib.cmake /usr/share/cmake-3.16/Modules/

# Build the package
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && git clone https://github.com/StarlingUAS/starling_simple_offboard.git /ros_ws/src/simple_offboard \
    && git clone https://github.com/StarlingUAS/starling_allocator.git /ros_ws/src/starling_allocator \
    && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
    && cd /ros_ws \
    && colcon build --packages-select simple_offboard_msgs starling_allocator_msgs\
    && rm -r build

# Build the messages
COPY synchronous_msgs /ros_ws/src/synchronous_msgs
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
    && cd /ros_ws \
    && colcon build --packages-select synchronous_msgs\
    && rm -r build

COPY .git /ros_ws/src/.git
COPY .gitmodules /ros_ws/src/.gitmodules

# Build the package
COPY position_trajectory_controller /ros_ws/src/position_trajectory_controller
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && . /ros_ws/install/setup.sh \
    && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
    && cd /ros_ws \
    && colcon build --packages-select position_trajectory_controller \
    && rm -r build

# Build the package
COPY sync_monitor /ros_ws/src/sync_monitor
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && . /ros_ws/install/setup.sh \
    && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
    && cd /ros_ws \
    && colcon build --packages-select sync_monitor \
    && rm -r build

COPY run.sh .

CMD ["./run.sh"]