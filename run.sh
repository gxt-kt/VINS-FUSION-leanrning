#!/bin/bash  


myecho() {  
  echo -e "\033[32m$1\033[0m"  
}  

# 执行 catkin_make 命令  
if ! catkin_make; then  
  myecho "catkin_make failed. Stopping script execution."  
  exit 1  
fi  
  
# catkin_make 成功后执行的代码  
myecho "catkin_make succeeded. Continuing script execution."  

  
# 定义中断信号处理函数  
interrupt_handler() {  
  # 杀死所有子进程  
  kill $(jobs -p) &>/dev/null  
  # 退出脚本  
  exit  
}
  
# 注册中断信号处理函数  
trap interrupt_handler SIGINT  
  
# 以下是你的脚本内容...  
source ./devel/setup.bash  
  
  
myecho "roslaunch vins vins_rviz.launch"  
  
roslaunch vins vins_rviz.launch &  
roslaunch_pid=$!  
  
# 等待，让roslaunch命令有足够的时间启动  
sleep 2
  
  
  
myecho "rosrun vins vins_node ./src/VINS-Fusion-DetailedNote/config/euroc/euroc_mono_imu_config.yaml"  
  
# rosrun vins vins_node ./src/VINS-Fusion-DetailedNote/config/euroc/euroc_mono_imu_config.yaml &  
FILE_DIR=$(dirname "$(realpath "$0")")
rosrun vins vins_node $FILE_DIR/src/VINS-Fusion-DetailedNote/config/euroc/euroc_stereo_config.yaml &  
  
sleep 2
  
myecho "rosbag play ./MH_01_easy.bag"  
rosbag play ./MH_01_easy.bag &>/dev/null
rosbag_pid=$!  
  
# 等待所有子进程结束  
wait $roslaunch_pid $rosbag_pid  
