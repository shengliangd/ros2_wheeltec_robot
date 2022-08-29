在 Docker 中构建[轮趣科技](https://wheeltec.net/)的 Mini ROS 小车 ROS2 软件包。

使用方法：

1. 下载该公司提供的 wheeltec-ros1-src.zip、wheeltec-ros2-src.zip、LSM10.rar，分别解压到 wheeltec-ros1-src、wheeltec-ros2-src、LSM10 这三个文件夹下；
2. 注释掉 LSM10 文件夹下两个 CMakeLists.txt 里包含 dynamic_reconfigure 的行；
2. 如果需要的话，替换 camera_info 中的文件；
3. 执行 `docker build . -t ros2_wheeltec_robot -f docker/Dockerfile`。

运行容器时可以参考 `bash docker/run.sh`。

对应 ros2 galactic 版本，2022 年 8 月 29 日有效；后续如果相关软件包有更新可能会构建失败。
