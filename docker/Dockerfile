FROM ros:galactic

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get install -y ros-$ROS_DISTRO-filters libuvc-dev ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-robot-localization ros-$ROS_DISTRO-rtabmap-ros ros-$ROS_DISTRO-depth-image-proc ros-$ROS_DISTRO-navigation2

WORKDIR /work/ros2_ws

COPY wheeltec-ros2-src/src src

# install dependencies, some may fail but practically it's ok
RUN bash -c 'source /ros_entrypoint.sh && rosdep install --from-paths src -r -y || true'

# some packages are not used, just skip them
ENV SKIP_PACKAGES="lsm10_v2"
# build!
# '|| true's are needed because some packages may fail to build for the first time
RUN bash -c 'source /ros_entrypoint.sh && colcon build --executor sequential --symlink-install --packages-skip $SKIP_PACKAGES' || true
RUN bash -c 'source /ros_entrypoint.sh && colcon build --executor sequential --symlink-install --packages-select wheeltec_rrt_msg --packages-skip $SKIP_PACKAGES' || true
RUN bash -c 'source /ros_entrypoint.sh && source install/setup.bash && colcon build --executor sequential --symlink-install --packages-select wheeltec_robot_rrt --packages-skip SKIP_PACKAGES' || true
RUN bash -c 'source /ros_entrypoint.sh && source install/setup.bash && colcon build --executor sequential --symlink-install --packages-skip $SKIP_PACKAGES'
ENV SKIP_PACKAGES=

# required for ros2_astra_camera
ENV LD_LIBRARY_PATH="/work/ros2_ws/src/ros2_astra_camera-main/ros2_astra_camera/lib/openni2_redist/arm64:${LD_LIBRARY_PATH}"

COPY camera_info /root/.ros/camera_info

ENV DEBIAN_FRONTEND=

WORKDIR /work
