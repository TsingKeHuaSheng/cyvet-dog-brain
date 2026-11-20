# ========================================================
#  Cyvet 机器狗 SDK 示例 —— 独立业务工程（不依赖 SDK 源码树）
#
#  在 Jetson（外部 ARM64 主机）上原生构建：
#    export UNIUBI_SDK_ROOT="$HOME/uniubi_robot_sdk"
#    cmake -S . -B build -DPLATFORM=aarch64_host
#    cmake --build build -j
#
#  为什么必须 PLATFORM=aarch64_host：
#    ARM64 默认会被当成狗的大脑板（Orin），去取 lib/aarch64/；
#    我们是"外部主机"，必须显式声明才取 lib/aarch64_host/。
# ========================================================

cmake_minimum_required(VERSION 3.18)

project(cyvet_dog_demo VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

# SDK 根目录：含 include/ 和 lib/<arch>/
if(NOT UNIUBI_SDK_ROOT AND DEFINED ENV{UNIUBI_SDK_ROOT})
    set(UNIUBI_SDK_ROOT "$ENV{UNIUBI_SDK_ROOT}")
endif()
if(NOT UNIUBI_SDK_ROOT)
    message(FATAL_ERROR
        "UNIUBI_SDK_ROOT 未设置。先 source ~/env_uniubi.sh，"
        "或 cmake -DUNIUBI_SDK_ROOT=$HOME/uniubi_robot_sdk")
endif()

# 目标架构 -> SDK 运行库子目录（和 SDK 自身 CMakeLists 的规则保持一致）
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
    set(UNIUBI_ARCH_DIR "x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
    if(PLATFORM STREQUAL "aarch64_host")
        set(UNIUBI_ARCH_DIR "aarch64_host")
    else()
        set(UNIUBI_ARCH_DIR "aarch64")
    endif()
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(i.86|x86)$")
    set(UNIUBI_ARCH_DIR "i386")
else()
    message(FATAL_ERROR
        "不支持的 CMAKE_SYSTEM_PROCESSOR='${CMAKE_SYSTEM_PROCESSOR}'；"
        "期望 x86_64 / aarch64 / i386")
endif()

set(UNIUBI_LIB_DIR "${UNIUBI_SDK_ROOT}/lib/${UNIUBI_ARCH_DIR}")
if(NOT EXISTS "${UNIUBI_LIB_DIR}")
    message(FATAL_ERROR "找不到运行库目录 ${UNIUBI_LIB_DIR}（架构 ${UNIUBI_ARCH_DIR} 不对？）")
endif()

# SDK 自带 CMake target 只在 SDK 源码树里存在；这里直接按目录找库，
# 这样业务工程可以直接落在任意位置，不必先 cmake --install。
foreach(_lib robotMotionSdk mediaBus ubase udbus)
    find_library(UNIUBI_LIB_${_lib}
        NAMES ${_lib}
        HINTS "${UNIUBI_LIB_DIR}"
        NO_DEFAULT_PATH
        REQUIRED)
endforeach()

add_executable(cyvet_dog_demo src/cyvet_dog_demo.cpp)

target_include_directories(cyvet_dog_demo PRIVATE "${UNIUBI_SDK_ROOT}/include")

target_link_libraries(cyvet_dog_demo PRIVATE
    ${UNIUBI_LIB_robotMotionSdk}
    ${UNIUBI_LIB_mediaBus}
    ${UNIUBI_LIB_ubase}
    ${UNIUBI_LIB_udbus}
    Threads::Threads)

# 让产物自带 RUNPATH，拷到同机其它目录也能直接跑，不必依赖 LD_LIBRARY_PATH
set_target_properties(cyvet_dog_demo PROPERTIES
    BUILD_WITH_INSTALL_RPATH ON
    INSTALL_RPATH "${UNIUBI_LIB_DIR}")

# 链接期在 SDK 库目录里解析 DT_NEEDED；目标机才有的共享库符号留到运行时加载
target_link_options(cyvet_dog_demo PRIVATE
    "-Wl,-rpath-link,${UNIUBI_LIB_DIR}"
    "-Wl,--allow-shlib-undefined")

message(STATUS "Uniubi SDK root : ${UNIUBI_SDK_ROOT}")
message(STATUS "Uniubi arch dir : ${UNIUBI_ARCH_DIR} (processor=${CMAKE_SYSTEM_PROCESSOR}, PLATFORM=${PLATFORM})")
message(STATUS "Uniubi lib dir  : ${UNIUBI_LIB_DIR}")

install(TARGETS cyvet_dog_demo RUNTIME DESTINATION bin)
