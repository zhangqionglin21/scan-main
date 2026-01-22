# 定义cmake最低版本
cmake_minimum_required(VERSION 3.27)

# 定义项目名称
project(lms)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)

# 添加库构建目录
add_subdirectory(src)
