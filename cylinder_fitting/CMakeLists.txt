cmake_minimum_required(VERSION 3.27)

# 项目名称
project(cylinder_fitting)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)

# 定义库名称变量
set(LIB_NAME cylinder_fitting)

# 添加源文件
set(SOURCES
        src/export.cpp
        src/algorithm.cpp
        src/cmath.cpp
        src/vec3.cpp
)

# 设置输出目录
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/dll)  # 设置动态库的输出目录
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/lib)  # 设置静态库的输出目录
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/bin)  # 设置可执行文件的输出目录

# 添加库
add_library(${LIB_NAME} STATIC ${SOURCES})
#add_library(${LIB_NAME} SHARED ${SOURCES})

# 设置头文件路径, 避免源文件中需要使用相对路径来包含文件
target_include_directories(${LIB_NAME} PRIVATE include)

# 设置输出的DLL名称
set_target_properties(${LIB_NAME} PROPERTIES OUTPUT_NAME ${LIB_NAME})

# 设置输出的头文件目录
set_target_properties(${LIB_NAME} PROPERTIES 
        PUBLIC_HEADER "export.h"
        PUBLIC_HEADER "vec3.h")

# 安装目标文件
install(TARGETS ${LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        PUBLIC_HEADER DESTINATION include)
install(FILES export.h DESTINATION include)


