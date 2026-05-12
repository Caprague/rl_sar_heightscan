移植到 Go2 自带 Jetson NX DevelopperKit 拓展坞上运行，编译修改记录。

1. 创建 conda 环境 ls_ws，使用 python 3.8 适配 Ros2 Foxy
2. 补安装 empy：pip install empy==3.3.4
3. 补安装 catkin 工具：pip install catkin-pkg
4. 补安装 lark：pip install lark
5. 补安装 numpy：pip install numpy
6. 补安装 python 3.8 和 3.9 的包：
	sudo apt-get update
	sudo apt-get install python3-dev python3.8-dev
	sudo apt-get install python3-dev python3.9-dev
7. 系统自带 CycloneDDS 版本与项目绑定的 CycloneDDS 版本不兼容，修改 CMakeLists，优先使用绑定的，解决方案概要：
在 `src/rl_sar/CMakeLists.txt` 中，为 `rl_real_go2` 和 `rl_real_g1` 添加：

```cmake
add_executable(rl_real_go2 src/rl_real_go2.cpp)
target_include_directories(rl_real_go2 BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/unitree_sdk2-2.0.0/thirdparty/include
    ${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/unitree_sdk2-2.0.0/thirdparty/include/ddscxx
)
target_link_libraries(rl_real_go2
    unitree_sdk2
    rl_sdk
    observation_buffer
    yaml-cpp
)
```

```cmake
add_executable(rl_real_g1 src/rl_real_g1.cpp)
target_include_directories(rl_real_g1 BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/unitree_sdk2-2.0.0/thirdparty/include
    ${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/unitree_sdk2-2.0.0/thirdparty/include/ddscxx
)
target_link_libraries(rl_real_g1
    unitree_sdk2
    rl_sdk
    observation_buffer
    yaml-cpp
)
```
	
