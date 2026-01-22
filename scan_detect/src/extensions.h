#ifndef SCAN_EXTENSIONS_H
#define SCAN_EXTENSIONS_H

// 点云相关的扩展
#include <utility>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>

/**
 * 点云相关操作
 */
namespace cloud {
    /**
     * @brief 获取点云的最小最大值
     * @param cloud 输入的点云 
     * @return 
     */
    std::pair<pcl::PointXYZ, pcl::PointXYZ> getMinMax(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    /**
     * @brief 获取点云的大小
     * @param min_point 
     * @param max_point 
     * @return 
     */
    std::tuple<float, float, float> getXyzSize(const pcl::PointXYZ &min_point, const pcl::PointXYZ &max_point);

    /**
     * @brief 获取点云的大小
     * @param cloud 输入的点云
     * @return 返回X、Y、Z方向的大小
     */
    std::tuple<float, float, float> getCloudSize(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);


    /**
     * @brief 获取点云中Z值最小的点
     * @param cloud 输入的点云
     * @return 返回Z值最小的点 
     */
    pcl::PointXYZ getMinZPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    /**
     * @brief 获取点云中Z值最高的点
     * @param cloud 输入的点云
     * @return 返回Z值最小的点 
     */
    pcl::PointXYZ getMaxZPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    /**
     * @brief 对点云进行裁剪
     * @param cloud_in 
     * @param min_x 
     * @param max_x 
     * @param min_y 
     * @param max_y 
     * @param min_z 
     * @param max_z 
     * @return 
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    cropCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float min_x, float max_x, float min_y, float max_y,
              float min_z, float max_z);

    /**
     * @brief 对点云进行裁剪
     * @param cloud_in 
     * @param min_point 
     * @param max_point 
     * @return 
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    cropCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointXYZ min_point, pcl::PointXYZ max_point);

    /**
     * @brief 对点云进行裁剪
     * @param cloud_in 输入的点云
     * @param min_x 最小的X坐标
     * @param max_x 最大的X坐标 
     * @return 
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    cropCloudX(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float min_x, float max_x, bool negative = false);

    /**
     * @brief 对点云进行裁剪
     * @param cloud_in 输入的点云
     * @param min_y 最小的Y坐标
     * @param max_y 最大的Y坐标 
     * @return 
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    cropCloudY(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float min_y, float max_y, bool negative = false);

    /**
     * @brief 对点云进行裁剪
     * @param cloud_in 输入的点云
     * @param min_z 最小的Z坐标
     * @param max_z 最大的Z坐标 
     * @return 
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    cropCloudZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float min_z, float max_z, bool negative = false);

    /**
     * 计算点云的平均值
     * @param cloud 输入的点云
     * @return 
     */
    pcl::PointXYZ getMean(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    /**
     * @brief 获取点云中离指定点最近的点
     * @param cloud  
     * @param searchPoint 
     * @return 
     */
    std::pair<bool, pcl::PointXYZ>
    findNearestPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointXYZ &searchPoint);

    /**
     * @brief 获取点云中Xy平面(Z取平均值)四个角上的点，并不是直接获取包围盒，而且或者实际存在点云上的四个角的点
     * @param cloud 输入的点云
     * @return 查找到的四个点
     */
    std::vector<pcl::PointXYZ> findXyAnglePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    /**
     * @brief 对点云进行平面拟合
     * @param cloud_in
     * @param distance_threshold
     * @return 
     */
    pcl::ModelCoefficients::Ptr
    fitPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float distance_threshold = 100);

    /**
     * @brief 对点云进行直线拟合
     * @param cloud_in 输入点云
     * @param distance_threshold 距离阈值
     * @return 
     */
    pcl::ModelCoefficients::Ptr
    fitLine(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float distance_threshold);

    /**
     * @brief 对点云进行聚类，并返回所有的聚类点云数据
     * @param cloud_in 
     * @param tolerance 
     * @param min_size 
     * @param max_size 
     * @return 
     */
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
    euclideanCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float tolerance, int min_size,
                     int max_size = std::numeric_limits<int>::max());

    /**
     * @brief 对点云进行聚类，并返回最大的聚类点云数据
     * @param cloud_in 输入的点云 
     * @param tolerance 距离阈值 
     * @param min_size 最小的点数 
     * @param max_size 最大的点数 
     * @return 
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    maxEuclideanCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float tolerance, int min_size,
                        int max_size = std::numeric_limits<int>::max());

    /**
     * @brief 对点云进行执行sortFilter
     * @param cloud_in 
     * @param mean_k 
     * @param std_dev 
     * @return 
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    sorFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, int mean_k, float std_dev);

    /**
     * @brief 对点云通过体素进行下采样
     * @param cloud_in  输入的点云
     * @param leaf_size 体素大小 
     * @return 返回下采样后的点云
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    downSample(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float leaf_size);

    /**
     * @brief 对点云进行缩放, 注意是直接将坐标点乘以缩放因子，并不会基于指定中心点进行缩放
     * @param cloud_in 输入的点云
     * @param scale 缩放系数
     * @return 返回缩放后的点云 
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    scaleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float scale);

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    offsetCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, float xscale, float yscale, float zscale);

    /**
     * @brief 合并点云
     * @param clouds 输入的点云
     * @return 返回合并后的点云 
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    mergeClouds(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds);

    /**
     * @brief 将点转换为点云
     * @param points 输入的点
     * @return 返回点云 
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    pointsToCloud(const std::vector<pcl::PointXYZ> &points);
    
    /**
     * @brief 将点云的X轴都压缩到0
     * @param cloud_in 输入的点云
     * @return 返回压缩后的点云 
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    compressX(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in);
    
    /**
     * @brief 将点云的Y轴都压缩到0
     * @param cloud_in 输入的点云
     * @return 返回压缩后的点云 
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    compressY(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in);

    /**
     * @brief 根据指定的X范围的步长，统计这个范围内的Z的最小值
     * @param cloud_in 输入的点云
     * @param x_step X的步长
     * @return 返回X以及Z的最小值
     */
    std::vector<std::pair<float, float>>
    calcXMinZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float x_step = 100);
    
    /**
     * @brief 根据指定的X范围的步长，查找minZ突变的X位置值以及对应的突变值
     */
    std::pair<float, float> findMutationMinZX(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float x_step = 100);
}

/**
 * 平面相关操作
 */
namespace plane {
    /**
     * @brief 根据平面方程裁剪点云
     * @param coefficients 平面方程参数
     * @param cloud_in 输入的点云 
     * @param min_distance 距离阈值 
     * @param negative 是否取反， 默认为false，取距离范围内的点云
     * @param is_signed 是否带符号，默认为false
     * @return 
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropCloudByPlanDistance(const pcl::ModelCoefficients::Ptr &coefficients,
                                                                const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                                                                float min_distance, float max_distance,
                                                                bool is_signed = false);

    /**
     * @brief 根据XY以及平面方程计算Z
     * @param coefficients
     * @param x
     * @param y
     */
    float calcZ(const pcl::ModelCoefficients::Ptr &coefficients, float x, float y);

    /// <summary>
    /// 裁剪平面下方的点云
    /// </summary>
    /// <param name="coefficients"></param>
    /// <param name="cloud_in"></param>
    /// <param name="offset"></param>
    /// <returns></returns>
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropCloudPlaneDown(const pcl::ModelCoefficients::Ptr& coefficients, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, float offset = 0);
}

/**
 * 直线相关的操作
 */
namespace line {
    /**
     * @brief 计算点到直线的距离
     * @param line 直线参数 
     * @param point 点 
     * @return 
     */
    double distance(const pcl::ModelCoefficients::Ptr &line, const pcl::PointXYZ &point);
}

/**
 * 角度处理相关
 */
namespace angle {
    /**
     * @brief 将角度值转换成-90到90范围内，比如179度应该转换成-1度
     * @param angle 
     * @return 
     */
    double clamp(double angle);
}

#endif //SCAN_EXTENSIONS_H
