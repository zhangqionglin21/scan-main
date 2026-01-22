#ifndef SCAN_TYPES_H
#define SCAN_TYPES_H

#include <string>
#include <pcl/impl/point_types.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>

// 最大最小点
using MinMaxPoint = std::pair<pcl::PointXYZ, pcl::PointXYZ>;

// 识别结果类型  TODO: 待确认具体含义
enum DetectType {
    COIL = 1, // 钢卷
    SADDLE = 2, // 鞍座
    CAR_HEAD_POINT = 3, // 车厢头部点（X、Y地址最小的）
    CAR_TAIL_POINT = 4, // 车厢尾部点（X、Y地址最大的）
};

// 识别结果: 对外的接口结构
struct DetectResult {
    // 序号
    int car_index;
    // 类型
//    DetectType data_type;
    int data_type;

    // X坐标值
    double x;
    // Y坐标
    double y;
    // Z坐标
    double z;

    // 宽度
    double width;
    // 直径
    double diameter;
    // 轴心角度: 单位度
    double axis;
};


// 定义坐标轴
enum Axis {
    X,
    Y,
    Z,
};

// 定义方向
enum AxisDirection {
    X_POSITIVE, // x正方向: 朝向x大地址方向
    X_NEGATIVE, // x负方向: 朝向x小地址方向
    Y_POSITIVE,
    Y_NEGATIVE,
    Z_POSITIVE,
    Z_NEGATIVE,
};

/**
 * 从字符串获取坐标带方向的坐标轴
 * @param axis 字符串
 * @return 
 */
AxisDirection getDirectionAxis(const std::string &axis);

/**
 * 将坐标带方向的坐标轴转换为字符串
 * @param axis 
 * @return 
 */
std::string getAxisString(AxisDirection axis);

/**
 * 从方向获取坐标轴
 * @param direction 方向
 * @return 
 */
Axis getAxis(AxisDirection direction);

/**
 * 从字符串获取坐标轴
 * @param axis 
 * @return 
 */
Axis getAxis(const std::string &axis);

// 空间点
struct Point {
    // x坐标
    float x;
    // y坐标
    float y;
    // z坐标
    float z;

    // 构造函数
    Point() : x(0), y(0), z(0) {}

    Point(float x, float y, float z) : x(x), y(y), z(z) {}

    explicit Point(pcl::PointXYZ point) : x(point.x), y(point.y), z(point.z) {}

public:
    // 重载类型转换
    explicit operator pcl::PointXYZ() const {
        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;
        return point;
    }

    // 重载等于判断
    bool operator==(const Point &other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    // 计算两点之间的距离
    static double distance(const Point &p1, const Point &p2) {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
    }

    // 计算两个点之间的距离
    [[nodiscard]] double distance(const Point &other) const {
        return distance(*this, other);
    }

    // 计算到原点的距离
    [[nodiscard]] double distance() const {
        return distance(*this, Point());
    }

    // 计算两个点在XY平面上的距离
    static double distanceXY(const Point &p1, const Point &p2) {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }

    // 计算两个点在XY平面上的距离
    [[nodiscard]] double distanceXY(const Point &other) const {
        return distanceXY(*this, other);
    }

    // 计算到XY平面原点的距离
    [[nodiscard]] double distanceXY() const {
        return distanceXY(*this, Point());
    }
};

// XY平面的直线信息
struct LineXy {
    // 点A 
    Point point_a;

    // 点B
    Point point_b;

    // (y-y1)/(y2-y1)=(x-x1)/(x2-x1) 
    LineXy()
    {

    }
    LineXy(Point point_a, Point point_b) : point_a(point_a), point_b(point_b) {

    }

    /**
     * @brief 根据x计算y
     * @param x 
     * @return 返回计算出来的y值 
     */
    [[nodiscard]] float getY(float x) const {
        auto y = ((point_b.y - point_a.y) * (x - point_a.x)) / (point_b.x - point_a.x) + point_a.y;
        return y;
    }

    /**
     * @brief 根据x计算y
     * @param x 
     * @return 返回计算出来的y值 
     */
    [[nodiscard]] float getX(float y) const {
        auto x = ((point_b.x - point_a.x) * (y - point_a.y)) / (point_b.y - point_a.y) + point_a.x;
        return x;
    }
};


// 平面四边形，主要用于表示车厢范围
struct PlaneQuadrilateral {
    // 顶点1
    Point min_min;
    // 顶点2
    Point min_max;
    // 顶点3
    Point max_min;
    // 顶点4
    Point max_max;

    // y小地址方向线段，用于表示车厢的长边
    LineXy y_min_line;

    // y大地址方向线段，用于表示车厢的长边
    LineXy y_max_line;


    // 构造函数
    PlaneQuadrilateral()
    {

    }
    PlaneQuadrilateral(Point min_min, Point min_max, Point max_min, Point max_max) :
            min_min(min_min), min_max(min_max), max_min(max_min), max_max(max_max),
            y_min_line(LineXy({min_min.x, min_min.y, min_min.z}, {max_min.x, max_min.y, max_min.z})),
            y_max_line(LineXy({min_max.x, min_max.y, min_max.z}, {max_max.x, max_max.y, max_max.z})) {}

    PlaneQuadrilateral(pcl::PointXYZ min_min, pcl::PointXYZ min_max, pcl::PointXYZ max_min, pcl::PointXYZ max_max) :
            min_min(min_min), min_max(min_max), max_min(max_min), max_max(max_max),
            y_min_line(LineXy({min_min.x, min_min.y, min_min.z}, {max_min.x, max_min.y, max_min.z})),
            y_max_line(LineXy({min_max.x, min_max.y, min_max.z}, {max_max.x, max_max.y, max_max.z})) {}

    /**
     * 在XY平面上缩放四边形指定的距离
     * @param distance 缩放的距离，如果为正数则放大，如果为负数则缩小
     * @return 
     */
    [[nodiscard]] PlaneQuadrilateral scaleDistanceXy(double distance) const {
        std::vector<Point> result;

        // 计算四边形中心点坐标
        Point center{};
        center.x = (min_min.x + min_max.x + max_min.x + max_max.x) / 4;
        center.y = (min_min.y + min_max.y + max_min.y + max_max.y) / 4;

        // 对每个顶点进行缩放计算
        for (const auto &point: {min_min, min_max, max_min, max_max}) {
            // 计算顶点与中心点的距离
            double dist = point.distance(center);

            // 计算缩放因子
            double scale = (dist + distance) / dist;

            // 计算新的顶点坐标
            Point newPoint{};
            newPoint.x = (float) (center.x + (point.x - center.x) * scale);
            newPoint.y = (float) (center.y + (point.y - center.y) * scale);
            newPoint.z = point.z;

            // 将新的顶点坐标添加到结果中
            result.push_back(newPoint);
        }

        return {result[0], result[1], result[2], result[3]};
    }

    /**
     * 获取X的范围
     */
    [[nodiscard]] std::pair<float, float> xRange() const {
        return std::make_pair(std::min(std::min(min_min.x, min_max.x), std::min(max_min.x, max_max.x)),
                              std::max(std::max(min_min.x, min_max.x), std::max(max_min.x, max_max.x)));
    }

    /**
     * 获取Y的范围
     */
    [[nodiscard]] std::pair<float, float> yRange() const {
        return std::make_pair(std::min(std::min(min_min.y, min_max.y), std::min(max_min.y, max_max.y)),
                              std::max(std::max(min_min.y, min_max.y), std::max(max_min.y, max_max.y)));
    }

    /**
     * @brief 获取最小最大点
     * @return 
     */
    [[nodiscard]] MinMaxPoint getMinMax() const {
        pcl::PointXYZ min_point, max_point;
        min_point.x = std::min(std::min(min_min.x, min_max.x), std::min(max_min.x, max_max.x));
        min_point.y = std::min(std::min(min_min.y, min_max.y), std::min(max_min.y, max_max.y));
        min_point.z = std::min(std::min(min_min.z, min_max.z), std::min(max_min.z, max_max.z));
        max_point.x = std::max(std::max(min_min.x, min_max.x), std::max(max_min.x, max_max.x));
        max_point.y = std::max(std::max(min_min.y, min_max.y), std::max(max_min.y, max_max.y));
        max_point.z = std::max(std::max(min_min.z, min_max.z), std::max(max_min.z, max_max.z));
        return {min_point, max_point};
    }
    
    /**
     * @brief 计算指定X的对应的车厢的Y的中心位置
     * @param x 
     * @return 
     */
    [[nodiscard]] float getCenterY(float x) const{
       auto min_y = y_min_line.getY(x);
       auto max_y = y_max_line.getY(x);
       return (min_y + max_y) / 2;
    }
    
    /**
     * 获取四个顶点的
     */
    [[nodiscard]] std::vector<Point> getPoints() const {
        return {min_min, min_max, max_min, max_max};
    }
};


// 车厢信息
struct Carriage {
    Carriage()
    {

    }
    Carriage(PlaneQuadrilateral inputborder,

    // 车厢平面的方程参数
    pcl::ModelCoefficients::Ptr inputplane,

    // 车厢的点云: 裁剪掉边框，包含车厢内部鞍座、钢卷的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud)
    {
        border = inputborder;
        plane = inputplane;
        cloud = inputcloud;
    }
    // 车厢的边界范围: 四边形，并非一定是矩形
    PlaneQuadrilateral border;

    // 车厢平面的方程参数
    pcl::ModelCoefficients::Ptr plane;

    // 车厢的点云: 裁剪掉边框，包含车厢内部鞍座、钢卷的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    /**
     * 获取车厢平面的中心点
     * @return 返回车厢平面的中心点的坐标，注意是车厢底面的而不是整个车厢的
     */
    [[nodiscard]] Point getPlaneCenter() const;
};

#endif //SCAN_TYPES_H
