if [ $# -eq 0 ]; then
    echo "usage: $0 <interface>"
    exit
fi
INTERFACE=$1
CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>$INTERFACE</NetworkInterfaceAddress></General></Domain></CycloneDDS>"

docker run -td --name ros2_wheeltec_robot -e DISPLAY=$DISPLAY -e CYCLONEDDS_URI=$CYCLONEDDS_URI -v /tmp/.X11-unix --network host --ipc host --privileged -v /dev:/dev -v `readlink -f ./wheeltec-ros2-src/src`:/work/ros2_ws/src ros2_wheeltec_robot bash
