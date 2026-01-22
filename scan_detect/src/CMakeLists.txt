cmake_minimum_required(VERSION 3.27)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)

# 定义库名称变量
set(LIB_NAME scan_detect)

# 查找PCL库
find_package(PCL 1.11.1 REQUIRED)

# 查找Boost库
find_package(Boost REQUIRED COMPONENTS system log)

# 添加头文件路径
include_directories(
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/fit_cylinder/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../cylinder_fitting/include
)

# 添加库路径
link_directories(
        ${PCL_LIBRARY_DIRS}
        ${Boost_LIBRARY_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/fit_cylinder/lib
)

# 获取目录下所有文件
file(GLOB_RECURSE ALL_SOURCE_FILES 
        ${CMAKE_CURRENT_SOURCE_DIR}/*
)

# 添加源文件
file(GLOB_RECURSE ALL_FITTING_SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/../../cylinder_fitting/src/*
)

# 添加库
add_library(${LIB_NAME} SHARED ${ALL_SOURCE_FILES} ${ALL_FITTING_SOURCE_FILES}) # 生成动态库

# 链接PCL库和Boost库
target_link_libraries(${LIB_NAME}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
#        ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/fit_cylinder/lib/fit_cylinder.lib
)
