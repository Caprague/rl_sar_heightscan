# rl_sar 编译报错：CycloneDDS 版本冲突问题分析与修复

## 背景

本项目 `rl_sar` 运行在 Unitree 机器人的 Jetson 平台上（Ubuntu 20.04, aarch64），使用 ROS2 Foxy 作为通信框架，同时依赖 `unitree_sdk2-2.0.0` 与机器人底层通信。

环境中存在两套 CycloneDDS：

| 组件 | 版本 | 路径 |
|------|------|------|
| ROS Foxy 系统安装 | **0.7.0** | `/opt/ros/foxy/include/dds/` |
| unitree_sdk2 bundled | **0.10.2** | `library/thirdparty/unitree_sdk2-2.0.0/thirdparty/include/dds/` |

`.bashrc` 中 `source /opt/ros/foxy/setup.bash` 会将 `/opt/ros/foxy` 加入 `CMAKE_PREFIX_PATH`，使系统 CycloneDDS 0.7.0 的头文件对 CMake 可见。

## 现象

编译 `rl_real_g1` 和 `rl_real_go2` 时出现大量错误，集中在 unitree_sdk2 bundled 的 ddscxx (C++ binding) 头文件中：

```
error: 'dds_topic_filter' has not been declared
error: 'DDS_TOPIC_FILTER_SAMPLE_ARG' was not declared in this scope
error: 'DDS_DATA_REPRESENTATION_FLAG_XCDR1' was not declared in this scope
error: 'dds_create_topic_sertype' was not declared in this scope
error: 'getSerType' is not a member of 'org::eclipse::cyclonedds::topic::TopicTraits<...>'
error: 'type' was not declared in this scope; did you mean 'strong::type'?
```

这些符号都是 CycloneDDS 0.10.x 新增的 C API，在 0.7.0 中不存在。

## 原因分析

### 头文件搜索顺序问题

1. unitree_sdk2 的 C++ headers (`ddscxx/`) 是为 CycloneDDS **0.10.2** 编写的，内部通过 `#include "dds/dds.h"` 引入底层 C API。

2. 编译器按照 include 路径顺序搜索头文件。由于 `source /opt/ros/foxy/setup.bash` 将 `/opt/ros/foxy` 加入了 `CMAKE_PREFIX_PATH`，CMake 的 `find_package` 机制使得 `/opt/ros/foxy/include` 出现在搜索路径中。

3. 当编译器解析 `#include "dds/dds.h"` 时，**优先找到了 ROS Foxy 系统的 0.7.0 版本**（`/opt/ros/foxy/include/dds/dds.h`），而非 bundled 的 0.10.2 版本。

4. 0.7.0 的 C API 缺少以下 0.10.x 新增内容：
   - `dds_topic_filter` 结构体（0.7 只有 `dds_topic_filter_fn` 函数指针类型）
   - `DDS_TOPIC_FILTER_SAMPLE_ARG` / `DDS_TOPIC_FILTER_SAMPLEINFO_ARG` 枚举值
   - `DDS_DATA_REPRESENTATION_FLAG_XCDR1` 宏
   - `dds_create_topic_sertype()` 函数（0.7 只有 `dds_create_topic_generic()`）
   - `TopicTraits<T>::getSerType()` 方法

5. 附带问题：libtorch 头文件中定义了 `strong::type`，与 CycloneDDS `datatopic.hpp:766` 中裸用的 `type` 变量产生命名冲突，这也是因为加载了错误版本的 C headers 导致编译上下文异常。

### 版本确认依据

- bundled `libddsc.so.0` 中的编译路径字符串：`cyclonedds-0.10.2/src/core/ddsi/src/...`
- bundled `thirdparty/include/dds/version.h`：`#define DDS_VERSION "0.10.2"`
- 系统 `/opt/ros/foxy/include/dds/version.h`：`#define DDS_VERSION "0.7.0"`

## 解决方案：使用 BEFORE 指令提升 include 优先级

### 原理

CMake 的 `target_include_directories` 支持 `BEFORE` 关键字，它会将指定路径插入到该 target 的 include 搜索列表**最前面**，优先于所有其他通过 `find_package`、`ament_target_dependencies` 等方式引入的系统路径。

这样编译器在解析 `#include "dds/dds.h"` 时，会先在 bundled 目录中找到 0.10.2 的头文件，不再触及系统的 0.7.0 版本。

### 具体修改

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

### 为什么需要两个路径

- `.../thirdparty/include`：包含 `dds/dds.h`（C API 主头文件）、`dds/version.h`、`dds/ddsc/` 等 C 层头文件
- `.../thirdparty/include/ddscxx`：包含 `dds/topic/`、`dds/sub/`、`org/eclipse/cyclonedds/` 等 C++ binding 头文件

两者配合才能让 bundled 的 C 和 C++ 头文件同时生效。

### 编译验证

```bash
rm -rf build/ install/ log/
colcon build --packages-select rl_sar
```
