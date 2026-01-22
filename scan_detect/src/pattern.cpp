#include "pattern.h"

/**
 * 查找最近的点
 * @param points 待查找的点集合
 * @param point 参考点
 * @return 匹配的点和索引
 */
std::pair<Point, int> pattern::findNearest(const std::vector<Point> &points, const Point &point) {
    if (points.empty()) {
        throw std::invalid_argument("points is empty");
    }

    auto min_distance = std::numeric_limits<double>::max();
    int min_index = 0;
    for (int i = 0; i < points.size(); i++) {
        auto distance = points[i].distance(point);
        if (distance < min_distance) {
            min_distance = distance;
            min_index = i;
        }
    }
    return std::make_pair(points[min_index], min_index);
}


/**
 * 获取匹配的点集合
 * @param points 待查找的点集合
 * @param pattern_points 参考点集合（对应的模式）
 * @return 匹配的点集合、对应的点的索引对（模式特征点-匹配上的点）、和匹配平均距离（越小越好） 
 */
std::pair<std::vector<std::pair<Point, Point>>, double>
pattern::getMatchPatternPoints(const std::vector<Point> &points, std::vector<Point> &pattern_points) {
    // 参数检查
    if (points.empty()) {
        throw std::invalid_argument("points is empty");
    }

    if (pattern_points.empty()) {
        throw std::invalid_argument("pattern_points is empty");
    }

    // 计算points的平均Y值
    auto points_mean = getMean(points);
    auto pattern_mean = getMean(pattern_points);

    // 计算偏移值
    // 查找X最小的点
    auto x_offset = getMinX(points);
    auto y_offset = points_mean.y - pattern_mean.y;
    auto z_offset = points_mean.z - pattern_mean.z;

    // 遍历points以其X作为偏移值
    auto best_match_points = std::vector<std::pair<Point, Point>>();
    auto best_match_distance = std::numeric_limits<double>::max();
//    for (const auto &point: points) {
    // 计算偏移后的点集合
    std::vector<std::pair<Point, Point>> offset_pattern_points;
    for (const auto &pattern_point: pattern_points) {
        auto offset_pattern_point = Point(pattern_point.x + x_offset, pattern_point.y + y_offset,
                                          pattern_point.z + z_offset);
        offset_pattern_points.emplace_back(offset_pattern_point, pattern_point);
    }

    // 遍历offset_pattern_points，查找最近的点，计算其与最近点的距离的平均值
    auto match_points = std::vector<std::pair<Point, Point>>();
    double match_distance = 0;
    for (auto &[offset_pattern_point, pattern_point]: offset_pattern_points) {
        // 查找最近的点
        auto [near_point, index] = findNearest(points, offset_pattern_point);

        // 计算距离
        match_distance += near_point.distance(offset_pattern_point);

        // 添加到匹配的点集合中: 需要添加原始的点
        match_points.emplace_back(pattern_point, near_point);
    }
    match_distance /= static_cast<double>(match_points.size());

    // 如果距离更小，则更新
    if (match_distance < best_match_distance) {
        best_match_points = match_points;
        best_match_distance = match_distance;
    }
//    }

    return std::make_pair(best_match_points, best_match_distance);
}


/**
 * 获取最佳匹配的模式
 * @param points 待查找的点集合
 * @param patterns 参考模式集合 
 * @return 最佳的模式的索引和匹配的点集合
 */
std::pair<int, std::vector<std::pair<Point, Point>>>
pattern::getBestMatchPattern(const std::vector<Point> &points, std::vector<std::vector<Point>> &patterns) {
    // 检查参数
    if (points.empty()) {
        throw std::invalid_argument("points is empty");
    }

    if (patterns.empty()) {
        throw std::invalid_argument("patterns is empty");
    }

    // 遍历patterns，获取最佳匹配的模式
    auto best_match_index = 0;
    auto best_match_points = std::vector<std::pair<Point, Point>>();
    auto best_match_distance = std::numeric_limits<double>::max();
    for (int i = 0; i < patterns.size(); i++) {
        // 判断模式匹配的分数
        auto [match_points_pair, match_distance] = getMatchPatternPoints(points, patterns[i]);

        // 如果距离更小，则更新
        if (match_distance < best_match_distance) {
            best_match_index = i;
            best_match_points = match_points_pair;
            best_match_distance = match_distance;
        }
    }

    // 临时输出 TODO: 
//    std::cout << "best_match_index: " << best_match_index << ", pattern count" << patterns.size()
//              << ", best_match_distance: " << best_match_distance << std::endl;

    // 返回结果
    return std::make_pair(best_match_index, best_match_points);
}


/**
 * 获取最佳匹配的模式，从所有的点中搜索出最佳多种组合
 * @param points 待查找的点集合 
 * @param patterns 参考模式集合 
 * @return 最佳的模式的索引和匹配的点集合 
 */
std::vector<std::pair<int, std::vector<std::pair<Point, Point>>>>
pattern::getBestMatchPatterns(const std::vector<Point> &points, std::vector<std::vector<Point>> &patterns) {
    // 检查参数
    if (points.empty()) return {};

    if (patterns.empty()) {
        throw std::invalid_argument("patterns is empty");
    }

    auto copy_points = copy(points);
    auto result = std::vector<std::pair<int, std::vector<std::pair<Point, Point>>>>();
    while (!copy_points.empty()) {
        // 尝试匹配
        auto [best_pattern_index, match_points] = getBestMatchPattern(copy_points, patterns);

        // 添加到结果中
        result.emplace_back(best_pattern_index, match_points);

        // 将剩余的点赋值给copy_points
        std::vector<Point> new_copy_points;
        for (const auto &point: copy_points) {
            bool is_match = false;
            for (const auto &match_point: match_points) {
                if (point == match_point.second) {
                    is_match = true;
                    break;
                }
            }
            if (!is_match) {
                new_copy_points.emplace_back(point);
            }
        }
        copy_points = new_copy_points;
    }

    return result;
}


/**
 * 获取点集合的平均值 
 * @param points 
 * @return 
 */
Point pattern::getMean(const std::vector<Point> &points) {
    if (points.empty()) {
        throw std::invalid_argument("points is empty");
    }

    Point result{};
    for (const auto &point: points) {
        result.x += point.x;
        result.y += point.y;
        result.z += point.z;
    }
    result.x /= static_cast<float>(points.size());
    result.y /= static_cast<float>(points.size());
    result.z /= static_cast<float>(points.size());
    
    return result;
}


/**
 * 获取最小的X地址
 * @param points 
 * @return 
 */
float pattern::getMinX(const std::vector<Point> &points) {
    if (points.empty()) {
        throw std::invalid_argument("points is empty");
    }

    auto min_x = std::numeric_limits<float>::max();
    Point result{};
    for (const auto &point: points) {
        if (point.x < min_x) {
            min_x = point.x;
            result = point;
        }
    }
    return result.x;
}

/**
 * 获取点集合的平均Y值
 * @param points 
 * @return 
 */
double pattern::getMeanY(const std::vector<Point> &points) {
    return getMean(points).y;
}

/**
 * 复制点集合
 * @param points 
 * @return 
 */
std::vector<Point> pattern::copy(const std::vector<Point> &points) {
    return std::vector<Point>(points);
}
