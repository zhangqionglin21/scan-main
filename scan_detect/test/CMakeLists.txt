# 使用动态库
set(Boost_USE_STATIC_LIBS OFF)

# 设置测试用例名称
set(TEST_NAME test_scan)

add_definitions(-DNOMINMAX)

# 查找PCL
find_package(PCL 1.11.1 REQUIRED)

# 查找单元测试框架
find_package(Boost REQUIRED COMPONENTS unit_test_framework system log)

# 获取目录
include_directories(
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/fit_cylinder/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
        ${CMAKE_CURRENT_SOURCE_DIR}/../../cylinder_fitting/include
)

# 添加库路径
link_directories(
        ${PCL_LIBRARY_DIRS}
        ${Boost_LIBRARY_DIRS}
#        ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/fit_cylinder/lib
)

# 获取目录下所有文件
file(GLOB_RECURSE ALL_TEST_SOURCE_FILES ./*.cpp)

# 获取目录下所有文件
file(GLOB_RECURSE ALL_SOURCE_FILES ../src/*)

file(GLOB_RECURSE ALL_FITTING_SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/../../cylinder_fitting/src/*
)

# 构造测试用例
add_executable(${TEST_NAME} ${ALL_TEST_SOURCE_FILES} ${ALL_SOURCE_FILES} ${ALL_FITTING_SOURCE_FILES})

# 链接测试程序
target_link_libraries(${TEST_NAME}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
#        scan_recognition
)

set(COPY_TARGET_NAME ${TEST_NAME})
get_filename_component(PARENT_DIR ${CMAKE_CURRENT_LIST_DIR} DIRECTORY)
include(${PARENT_DIR}/cmake/CopyLibraries.cmake)

