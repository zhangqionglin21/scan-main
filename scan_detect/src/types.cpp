#include <algorithm>
#include <stdexcept>
#include "types.h"
#include "extensions.h"

/**
 * 从字符串获取坐标带方向的坐标轴
 * @param axis 字符串
 * @return 
 */
AxisDirection getDirectionAxis(const std::string &axis) {
    // 去除前后空白字符
    std::string axis_trim = axis;
    axis_trim.erase(0, axis_trim.find_first_not_of(' '));
    axis_trim.erase(axis_trim.find_last_not_of(' ') + 1);

    // 转换成小写
    std::transform(axis_trim.begin(), axis_trim.end(), axis_trim.begin(), std::tolower);

    if (axis_trim == "x") {
        return X_POSITIVE;
    } else if (axis_trim == "-x") {
        return X_NEGATIVE;
    } else if (axis_trim == "y") {
        return Y_POSITIVE;
    } else if (axis_trim == "-y") {
        return Y_NEGATIVE;
    } else if (axis_trim == "z") {
        return Z_POSITIVE;
    } else if (axis_trim == "-z") {
        return Z_NEGATIVE;
    } else {
        throw std::invalid_argument("Invalid axis: " + axis);
    }
}

std::string getAxisString(AxisDirection axis) {
    switch (axis) {
        case X_POSITIVE:
            return "X";
        case X_NEGATIVE:
            return "-X";
        case Y_POSITIVE:
            return "Y";
        case Y_NEGATIVE:
            return "-Y";
        case Z_POSITIVE:
            return "Z";
        case Z_NEGATIVE:
            return "-Z";
        default:
            throw std::invalid_argument("Invalid axis");
    }
}

/**
 * 从方向获取坐标轴
 * @param direction 方向
 * @return 
 */
Axis getAxis(AxisDirection direction) {
    switch (direction) {
        case X_POSITIVE:
        case X_NEGATIVE:
            return X;
        case Y_POSITIVE:
        case Y_NEGATIVE:
            return Y;
        case Z_POSITIVE:
        case Z_NEGATIVE:
            return Z;
        default:
            throw std::invalid_argument("Invalid direction");
    }
}

/**
 * 从字符串获取坐标轴
 * @param axis 
 * @return 
 */
Axis getAxis(const std::string &axis) {
    return getAxis(getDirectionAxis(axis));
}

/**
 * 获取车厢平面的中心点
 * @return 返回车厢平面的中心点的坐标，注意是车厢底面的而不是整个车厢的
 */
Point Carriage::getPlaneCenter() const {
    auto [min_point, max_point] = border.getMinMax();

    // 根据点云范围计算中心点的x和y
    auto center_x = (min_point.x + max_point.x) / 2;
    auto center_y = (min_point.y + max_point.y) / 2;

    // 根据x、y以及平面方程计算对应的z
    auto plane_z = plane::calcZ(plane, center_x, center_y);

    return {center_x, center_y, plane_z};
}
