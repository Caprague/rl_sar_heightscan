# 这段变量设置过多，且直接暴露在 bashrc 中，环境管理混乱，已经注释了一大堆，以避免影响编译
# >>> fishros initialize >>>
echo "ros:foxy(1) noetic(2) ?"
read choose
case $choose in
1) source  /opt/ros/foxy/setup.bash;

# terrible env vars management ...
#source ~/cyclonedds_ws/install/setup.bash;
#source /home/unitree/humanoid-robot-basic-cpp/install/setup.bash;
#export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp;
#export CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml;

export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH;;
2) source  /opt/ros/noetic/setup.bash;;
esac
# <<< fishros initialize <<<


# 如下是新加的功能函数，部分设为自动执行
#--------------------------------------------------------------------
cuda_env_set(){
        export PATH=/usr/local/cuda-11.4/bin:$PATH
        export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64:$LD_LIBRARY_PATH
}

libtorch_env_set(){
        export Torch_DIR=/home/unitree/package/libtorch_arm64/libtorch
}

cyclonedds_env_set(){
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
}

# export envs
# if not need, comment blow
cuda_env_set
libtorch_env_set

#--------------------------------------------------------------------
foxy_ros_source() {
    source /opt/ros/foxy/setup.bash
    echo "source foxy setup.bash OK !"

    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
        echo "source local install/setup.bash OK !"
    else
        echo "Warning: install/setup.bash not found, skipping..."
    fi
}

#--------------------------------------------------------------------
clash_setup_proxy() {
    export http_proxy=http://127.0.0.1:7897
    export https_proxy=http://127.0.0.1:7897
    export no_proxy=127.0.0.1,localhost
    export HTTP_PROXY=http://127.0.0.1:7897
    export HTTPS_PROXY=http://127.0.0.1:7897
    export NO_PROXY=127.0.0.1,localhost
    export all_proxy=socks5://127.0.0.1:7897
    echo -e "\033[32m[√] 已开启代理\033[0m"
}
clash_unset_proxy() {
    unset http_proxy
    unset https_proxy
    unset no_proxy
    unset HTTP_PROXY
    unset HTTPS_PROXY
    unset NO_PROXY
    unset all_proxy
    echo -e "\033[31m[×] 已关闭代理\033[0m"
}

#--------------------------------------------------------------------

