FROM ros:galactic

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update

WORKDIR /work/ros2_ws

RUN apt-get install -y ros-$ROS_DISTRO-filters
RUN apt-get install -y libuvc-dev
RUN apt-get install -y ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-robot-localization ros-$ROS_DISTRO-rtabmap-ros

# copy source code
COPY wheeltec-ros2-src/src src

# install dependencies
RUN bash -c 'source /ros_entrypoint.sh && rosdep install --from-paths src -r -y || true'

# LSM10
COPY LSM10 /tmp/LSM10
COPY LSM10/1-CMakeLists.txt src/wheeltec_lidar_ros2/LSlidar/lsm10_v2/CMakeLists.txt
RUN bash -c 'source /ros_entrypoint.sh && colcon build --symlink-install --packages-select lsm10_v2'
COPY LSM10/2-CMakeLists.txt src/wheeltec_lidar_ros2/LSlidar/lsm10_v2/CMakeLists.txt
RUN bash -c 'source /ros_entrypoint.sh && colcon build --symlink-install --packages-select lsm10_v2'

# build!
RUN bash -c 'source /ros_entrypoint.sh && colcon build --executor sequential --symlink-install --packages-skip nav2_system_tests' || true
RUN bash -c 'source /ros_entrypoint.sh && colcon build --executor sequential --symlink-install --packages-select wheeltec_rrt_msg --packages-skip nav2_system_tests' || true
ENV MAKEFLAGS="-j2"
RUN bash -c 'source /ros_entrypoint.sh && source install/setup.bash && colcon build --executor sequential --symlink-install --packages-select wheeltec_robot_rrt --packages-skip nav2_system_tests' || true
RUN bash -c 'source /ros_entrypoint.sh && source install/setup.bash && colcon build --executor sequential --symlink-install --packages-skip nav2_system_tests'
ENV MAKEFLAGS=

# ros1 packages
# WORKDIR /work/ros1_ws
##  COPY wheeltec-ros1-src/src src
# RUN DEBIAN_FRONTEND=noninteractive bash -c 'source /opt/ros/$ROS1_DISTRO/setup.bash && rosdep install --from-paths src -r -y'
# RUN bash -c 'source /opt/ros/$ROS1_DISTRO/setup.bash && catkin_make'
# RUN apt-get install -y ros-$ROS1_DISTRO-rgbd-launch

ENV LD_LIBRARY_PATH="/work/ros2_ws/src/ros2_astra_camera-main/ros2_astra_camera/lib/openni2_redist/arm64:${LD_LIBRARY_PATH}"

COPY camera_info /root/.ros/camera_info

ENV DEBIAN_FRONTEND=

WORKDIR /work
