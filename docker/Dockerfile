FROM ros:galactic-ros1-bridge

RUN apt-get update

WORKDIR /work/ros2_ws

# copy source code
COPY wheeltec-ros2-src/src src

# install dependencies
RUN DEBIAN_FRONTEND=noninteractive bash -c 'source /ros_entrypoint.sh && rosdep install --from-paths src -r -y'

# LSM10
COPY LSM10 /tmp/LSM10
RUN cp /tmp/LSM10/1-CMakeLists.txt src/wheeltec_lidar_ros2/LSlidar/lsm10_v2/CMakeLists.txt
RUN bash -c 'source /ros_entrypoint.sh && colcon build --packages-select lsm10_v2'
RUN cp /tmp/LSM10/2-CMakeLists.txt src/wheeltec_lidar_ros2/LSlidar/lsm10_v2/CMakeLists.txt
RUN bash -c 'source /ros_entrypoint.sh && colcon build --packages-select lsm10_v2'

# build!
RUN apt-get install -y ros-$ROS2_DISTRO-filters
RUN bash -c 'source /ros_entrypoint.sh && colcon build' || true
RUN bash -c 'source /ros_entrypoint.sh && colcon build --packages-select wheeltec_rrt_msg' || true
RUN bash -c 'source /ros_entrypoint.sh && source install/setup.bash && colcon build --packages-select wheeltec_robot_rrt' || true
# fine, let's manually set the ROS1 version for now
RUN apt-get install -y ros-$ROS1_DISTRO-libuvc-ros
RUN bash -c 'source /ros_entrypoint.sh && source install/setup.bash && colcon build --packages-skip nav2_system_tests'

COPY camera_info /root/.ros/
