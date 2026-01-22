cmake_minimum_required(VERSION 3.27)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)

# 定义库名称变量
set(LIB_NAME LMS3DSCAN)

# 添加头文件路径
include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

# 添加库路径
link_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/../bin/x64_release
)

# 查找链接库
find_library(LMS3D_LIB
        NAMES LMS3DScanPlatformDll_x64_210
        PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../bin/x64_release
)

# 输出目录
message(STATUS "LMS3D_LIB: ${LMS3D_LIB}")

# 获取目录下所有文件
file(GLOB_RECURSE ALL_SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/*
)

# 添加库
add_library(${LIB_NAME} SHARED ${ALL_SOURCE_FILES}) # 生成动态库

# 链接
target_link_libraries(${LIB_NAME} ${LMS3D_LIB})
