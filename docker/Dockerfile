FROM ros:galactic-ros1-bridge

RUN apt-get update

WORKDIR /work/ros2_ws

# copy source code
COPY wheeltec-ros2-src/src src

# install dependencies
RUN DEBIAN_FRONTEND=noninteractive bash -c 'source /ros_entrypoint.sh && rosdep install --from-paths src -r -y'

# LSM10
COPY LSM10 /tmp/LSM10
# TODO: comment dynamic_reconfigure
RUN cp /tmp/LSM10/1-CMakeLists.txt src/wheeltec_lidar_ros2/LSlidar/lsm10_v2/CMakeLists.txt
RUN bash -c 'source /ros_entrypoint.sh && colcon build --packages-select lsm10_v2'
RUN cp /tmp/LSM10/2-CMakeLists.txt src/wheeltec_lidar_ros2/LSlidar/lsm10_v2/CMakeLists.txt
RUN bash -c 'source /ros_entrypoint.sh && colcon build --packages-select lsm10_v2'

# build!
RUN apt-get install -y ros-$ROS2_DISTRO-filters
RUN bash -c 'source /ros_entrypoint.sh && colcon build' || true
RUN bash -c 'source /ros_entrypoint.sh && colcon build --packages-select wheeltec_rrt_msg' || true
RUN bash -c 'source /ros_entrypoint.sh && source install/setup.bash && colcon build --packages-select wheeltec_robot_rrt' || true
RUN apt-get install -y ros-$ROS1_DISTRO-libuvc-ros
RUN bash -c 'source /ros_entrypoint.sh && source install/setup.bash && colcon build --packages-skip nav2_system_tests'

RUN apt-get install -y ros-$ROS2_DISTRO-joint-state-publisher ros-$ROS2_DISTRO-robot-localization ros-$ROS2_DISTRO-rtabmap-ros

# fix camera product id, depends on your device
RUN sed -i 's/    product: "0x0502"/    product: "0x0501"/' src/ros2_astra_camera-main/ros2_astra_camera/launch/params/astra_pro.yaml
RUN bash -c 'source /ros_entrypoint.sh && source install/setup.bash && colcon build --packages-select ros2_astra_camera'

# ros1 packages
# WORKDIR /work/ros1_ws
##  COPY wheeltec-ros1-src/src src
# RUN DEBIAN_FRONTEND=noninteractive bash -c 'source /opt/ros/$ROS1_DISTRO/setup.bash && rosdep install --from-paths src -r -y'
# RUN bash -c 'source /opt/ros/$ROS1_DISTRO/setup.bash && catkin_make'
# RUN apt-get install -y ros-$ROS1_DISTRO-rgbd-launch

COPY camera_info /root/.ros/camera_info

WORKDIR /work
