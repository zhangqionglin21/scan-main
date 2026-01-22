# 定义cmake最低版本
cmake_minimum_required(VERSION 3.27)

# 定义项目名称
project(scan_detect)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)

# 查找Boost库
find_package(PCL 1.11.1 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system log)

# 添加库构建目录
add_subdirectory(src)

# 添加可执行文件构建目录
add_subdirectory(run)

# 添加测试目录
add_subdirectory(test)
