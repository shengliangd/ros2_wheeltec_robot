1、lsm10_v2功能包是在Linux ROS2环境中使用。该软件包在Ubuntu18.04上使用ROS2 Eloquent版本进行了测试。
2、这是m10雷达驱动程序包，在将包克隆到工作区后，使用colcon build --packages-select lsm10_v2 进行编译。
编译说明：
第一次编译时，需要将lsm10_v2功能包中的CMakeLists.txt文件内容替换成1-CMakeLists.txt文件后再进行编译；
编译完成后，再将CMakeLists.txt文件内容替换成2-CMakeLists.txt的内容编译即可。

3、发布话题：
``/scan`` (`sensor_msgs/scan`)
``/difop_information`` 
4、启动节点
ros2 launch lsm10_v2 ls_m10.launch.py

