#ifndef SCAN_PATTERN_H
#define SCAN_PATTERN_H

// 主要提供对鞍座匹配的类似模式匹配相关特定算法实现
#include <utility>
#include "types.h"

namespace pattern {
    /**
     * 查找最近的点
     * @param points 待查找的点集合
     * @param point 参考点
     * @return 匹配的点和索引
     */
    std::pair<Point, int> findNearest(const std::vector<Point> &points, const Point &point);

    /**
     * 获取匹配的点集合
     * @param points 待查找的点集合
     * @param pattern_points 参考点集合（对应的模式）
     * @return 匹配的点集合、对应的点的索引对（模式特征点-匹配上的点）、和匹配平均距离（越小越好） 
     */
    std::pair<std::vector<std::pair<Point, Point>>, double>
    getMatchPatternPoints(const std::vector<Point> &points, std::vector<Point> &pattern_points);

    /**
     * 获取最佳匹配的模式
     * @param points 待查找的点集合
     * @param patterns 参考模式集合 
     * @return 最佳的模式的索引和匹配 对应的点的索引对（模式特征点-匹配上的点）、和匹配平均距离（越小越好） 
     */
    std::pair<int, std::vector<std::pair<Point, Point>>>
    getBestMatchPattern(const std::vector<Point> &points, std::vector<std::vector<Point>> &patterns);

    /**
     * 获取最佳匹配的模式，从所有的点中搜索出最佳多种组合
     * @param points 待查找的点集合 
     * @param patterns 参考模式集合 
     * @return 最佳的模式的索引和匹配的点集合 
     */
    std::vector<std::pair<int, std::vector<std::pair<Point, Point>>>>
    getBestMatchPatterns(const std::vector<Point> &points, std::vector<std::vector<Point>> &patterns);

    /**
     * 获取点集合的平均值
     * @param points 
     * @return 
     */
    Point getMean(const std::vector<Point> &points);
    
    /**
     * 获取最小的X地址
     * @param points 
     * @return 
     */
    float getMinX(const std::vector<Point> &points);

    /**
     * 获取点集合的平均Y值
     * @param points 
     * @return 
     */
    double getMeanY(const std::vector<Point> &points);

    /**
     * 复制点集合
     * @param points 
     * @return 
     */
    std::vector<Point> copy(const std::vector<Point> &points);
}


#endif //SCAN_PATTERN_H
