docker run -td --name ros2_wheeltec_robot -e DISPLAY=$DISPLAY -v /tmp/.X11-unix --network host --ipc host --privileged -v /dev:/dev ros2_wheeltec_robot bash
