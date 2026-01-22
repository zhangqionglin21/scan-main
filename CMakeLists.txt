cmake_minimum_required(VERSION 3.31)

# 项目名称
project(scan)

# C++版本
set(CMAKE_CXX_STANDARD 17)

# 圆柱体拟合
add_subdirectory(cylinder_fitting)

# 扫描识别算法项目
add_subdirectory(scan_detect)

# lms的210的C++转C语言方式导出方便C#使用
add_subdirectory(lms)


