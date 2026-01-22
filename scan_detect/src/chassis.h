#ifndef SCAN_DETECT_CHASSIS_H
#define SCAN_DETECT_CHASSIS_H

#include "types.h"

// 架子信息
class Chassis {
public:
    // 架子的特征点
    std::vector<Point> features;

    // 架子对应的中心位置
    std::vector<Point> centers;

    /**
     * 根据匹配的的点集合，计算中心位置
     * @param match_points 匹配的坐标点对（特征点 - 实际点）
     * @return 
     */
    std::vector<Point> calcCenter(const std::vector<std::pair<Point, Point>> &match_points) {
        // 检查参数
        if (features.empty() || match_points.empty()) {
            return {};
        }

        // 依次计算两个点的偏移值
        std::vector<Point> offsets;
        // 记录距离, 用于记录标准差
        double sum_distances = 0;
        for (const auto &point: match_points) {
            auto offset = Point(point.second.x - point.first.x, point.second.y - point.first.y,
                                point.second.z - point.first.z);
            offsets.emplace_back(offset);

            // 计算标准差
            auto distance = offset.distance();
            sum_distances += distance * distance;
        }
        // 计算标准差
        auto std_dev = std::sqrt(sum_distances / static_cast<int>(match_points.size()));

        // 剔除大于2倍标准差的点
        auto mean_offset = Point();
        for (const auto &offset: offsets) {
            if (offset.distance() < 2 * std_dev) {
                mean_offset.x += offset.x;
                mean_offset.y += offset.y;
                mean_offset.z += offset.z;
            }
        }
        mean_offset.x /= static_cast<float>(offsets.size());
        mean_offset.y /= static_cast<float>(offsets.size());
        mean_offset.z /= static_cast<float>(offsets.size());

        // 将中心点统一偏移
        std::vector<Point> offset_centers;
        for (const auto &center: centers) {
            offset_centers.emplace_back(center.x + mean_offset.x, center.y + mean_offset.y, center.z + mean_offset.z);
        }

        // 计算中心位置
        return offset_centers;
    }


    /**
     * @brief 获取特征点
     * @param chassis 
     * @return 
     */
    static std::vector<std::vector<Point>> getFeatures(const std::vector<Chassis> &chassis) {
        std::vector<std::vector<Point>> features;
        for (const auto &ch: chassis) {
            features.push_back(ch.features);
        }
        return features;
    }

private:
    /**
     * 计算当前的点和xy平面原点最近的点
     * @param points 
     * @return 
     */
    static Point getNearPointToXyOrigin(const std::vector<Point> &points) {
        if (points.empty()) {
            throw std::invalid_argument("points is empty");
        }

        // 从特征点中查找距离原点最近的点: 直接计算距离
        auto near_point = points[0];
        for (const auto &point: points) {
            if (point.distanceXY() < near_point.distanceXY()) {
                near_point = point;
            }
        }
        return near_point;
    }
};




#endif //SCAN_DETECT_CHASSIS_H
