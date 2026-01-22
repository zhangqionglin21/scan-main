# 定义cmake最低版本
cmake_minimum_required(VERSION 3.27)

# 定义项目名称
set(RUN_NAME scan_bin)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)

# 查找Boost库
find_package(PCL 1.11.1 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system log)

# 添加头文件路径
include_directories(
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/fit_cylinder/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/
)

# 添加库路径
link_directories(
        ${PCL_LIBRARY_DIRS}
        ${Boost_LIBRARY_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/scan_recognition/lib
)


# 获取目录下所有文件
file(GLOB_RECURSE ALL_SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
)

# 添加源文件
set(SOURCES ${ALL_SOURCE_FILES})

# 添加可执行文件
add_executable(${RUN_NAME} main.cpp ${SOURCES})

target_link_libraries(${RUN_NAME}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
#        scan_recognition
)

# 拷贝文件
set(COPY_TARGET_NAME ${RUN_NAME})
get_filename_component(PARENT_DIR ${CMAKE_CURRENT_LIST_DIR} DIRECTORY)
include(${PARENT_DIR}/cmake/CopyLibraries.cmake)
