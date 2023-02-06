在 Docker 容器中构建[轮趣科技](https://wheeltec.net/)的 Mini ROS 小车 ROS2 软件包。

## 使用方法

1. 下载该公司提供的 ROS2 源码包，解压后移动到该目录下并重命名为 `wheeltec-ros2-src`（由于源码包体积较大且持续更新，完整源码包不包含在本仓库中）；
2. 根据 Wheeltec 提供的手册对涉及车型、Lidar 型号、相机型号等的文件进行修改；
3. 如有需要，生成新的相机参数文件并替换 camera_info 中的文件；
4. 执行 `docker build . -t ros2_wheeltec_robot -f docker/Dockerfile`。

创建容器时可以参考 `bash docker/create.sh` 中的命令，其中包含了常用的参数，包括指定多机通信的网卡、映射本地设备、禁用数据包多播以避免无线网络最低速率传输占满信道等。

## 注意事项

该公司的源码包持续更新，对于更新版本的源码包，可能需要修改 `docker/Dockerfile`；当前 Dockerfile 对于 20220829 版本有效。
