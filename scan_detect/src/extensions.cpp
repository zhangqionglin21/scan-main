#define  _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS

#include "extensions.h"

#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/distances.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>


std::pair<pcl::PointXYZ, pcl::PointXYZ> cloud::getMinMax(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    pcl::PointXYZ min_point, max_point;
    pcl::getMinMax3D(*cloud, min_point, max_point);

    return {min_point, max_point};
}


std::tuple<float, float, float> cloud::getXyzSize(const pcl::PointXYZ &min_point, const pcl::PointXYZ &max_point) {
    return {max_point.x - min_point.x, max_point.y - min_point.y, max_point.z - min_point.z};
}

std::tuple<float, float, float> cloud::getCloudSize(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    auto [min_point, max_point] = getMinMax(cloud);
    return getXyzSize(min_point, max_point);
}


/**
 * @brief 获取点云中Z值最小的点
 * @param cloud 输入的点云
 * @return 返回Z值最小的点 
 */
pcl::PointXYZ cloud::getMinZPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    auto lowest_point = cloud->points[0];
    for (auto &point: cloud->points) {
        if (point.z < lowest_point.z) {
            lowest_point = point;
        }
    }
    return lowest_point;
}


/**
 * @brief 获取点云中Z值最高的点
 * @param cloud 输入的点云
 * @return 返回Z值最小的点 
 */
pcl::PointXYZ cloud::getMaxZPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    auto highest_point = cloud->points[0];
    for (auto &point: cloud->points) {
        if (point.z > highest_point.z) {
            highest_point = point;
        }
    }
    return highest_point;
}

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
cloud::cropCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float min_x, float max_x, float min_y,
                 float max_y, float min_z, float max_z) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(min_x, max_x);
    pass.filter(*cloud_filtered);

    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(min_y, max_y);
    pass.filter(*cloud_filtered);

    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_z, max_z);
    pass.filter(*cloud_filtered);

    return cloud_filtered;
}

/**
 * @brief 对点云进行裁剪
 * @param cloud_in 
 * @param min_point 
 * @param max_point 
 * @return 
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr
cloud::cropCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointXYZ min_point,
                 pcl::PointXYZ max_point) {
    return cropCloud(cloud_in, min_point.x, max_point.x, min_point.y, max_point.y, min_point.z, max_point.z);
}


/**
 * @brief 对点云进行裁剪
 * @param cloud_in 输入的点云
 * @param min_x 最小的X坐标
 * @param max_x 最大的X坐标 
 * @return 
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr
cloud::cropCloudX(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float min_x, float max_x, bool negative) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(min_x, max_x);
    pass.setNegative(negative);  // 保留还是删除
    pass.filter(*cloud_filtered);
    return cloud_filtered;
}


/**
 * @brief 对点云进行裁剪
 * @param cloud_in 输入的点云
 * @param min_y 最小的Y坐标
 * @param max_y 最大的Y坐标 
 * @return 
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr
cloud::cropCloudY(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float min_y, float max_y, bool negative) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(min_y, max_y);
    pass.setNegative(negative);  // 保留还是删除
    pass.filter(*cloud_filtered);
    return cloud_filtered;
}

/**
 * @brief 对点云进行裁剪
 * @param cloud_in 输入的点云
 * @param min_z 最小的Z坐标
 * @param max_z 最大的Z坐标 
 * @return 
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr
cloud::cropCloudZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float min_z, float max_z, bool negative) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_z, max_z);
    pass.setNegative(negative);  // 保留还是删除
    pass.filter(*cloud_filtered);
    return cloud_filtered;
}

/**
 * @brief 计算点云的平均值
 * @param cloud 
 * @return 
 */
pcl::PointXYZ cloud::getMean(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    Eigen::Vector4f mean_point;
    pcl::compute3DCentroid(*cloud, mean_point);
    return pcl::PointXYZ{mean_point[0], mean_point[1], mean_point[2]};
}

/**
 * 执行sor滤波
 * @param cloud_in 输入的点云
 * @param mean_k 
 * @param std_dev 
 * @return 
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr
cloud::sorFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, int mean_k, float std_dev) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // TODO: 目前从测试看，如果点云数量小于等于mean_k，会导致程序崩溃，所以直接返回
    if (cloud_in->size() <= mean_k) {
        pcl::copyPointCloud(*cloud_in, *cloud_filtered);
        return cloud_filtered;
    }

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_filter;
    sor_filter.setInputCloud(cloud_in);
    sor_filter.setMeanK(mean_k);
    sor_filter.setStddevMulThresh(std_dev);
    sor_filter.filter(*cloud_filtered);

    return cloud_filtered;
}

/**
 * @brief 对点云通过体素进行下采样
 * @param cloud_in  输入的点云
 * @param leaf_size 体素大小 
 * @return 返回下采样后的点云
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr
cloud::downSample(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float leaf_size) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud_in);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*cloud_filtered);
    return cloud_filtered;
}


/**
 * @brief 对点云进行缩放, 注意是直接将坐标点乘以缩放因子，并不会基于指定中心点进行缩放
 * @param cloud_in 输入的点云
 * @param scale 缩放系数
 * @return 返回缩放后的点云 
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr
cloud::scaleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float scale) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scaled(new pcl::PointCloud<pcl::PointXYZ>);
    // 克隆点云，以尽量保持其长宽尺寸, 避免变成一维
    pcl::copyPointCloud(*cloud_in, *cloud_scaled);

    for (auto &point: cloud_scaled->points) {
        point.x *= scale;
        point.y *= scale;
        point.z *= scale;
    }
    return cloud_scaled;
}


/**
 * @brief 对点云进行缩放, 注意是直接将坐标点乘以缩放因子，并不会基于指定中心点进行缩放
 * @param cloud_in 输入的点云
 * @param scale 缩放系数
 * @return 返回缩放后的点云
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr
cloud::offsetCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, float xscale, float yscale, float zscale) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scaled(new pcl::PointCloud<pcl::PointXYZ>);
    // 克隆点云，以尽量保持其长宽尺寸, 避免变成一维
    pcl::copyPointCloud(*cloud_in, *cloud_scaled);

    for (auto& point : cloud_scaled->points) {
        point.x += xscale;
        point.y += yscale;
        point.z += zscale;
    }
    return cloud_scaled;
}

/**
 * @brief 合并点云
 * @param clouds 输入的点云
 * @return 返回合并后的点云 
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr
cloud::mergeClouds(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_merged(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &cloud: clouds) {
        *cloud_merged += *cloud;
    }
    return cloud_merged;
}


/**
 * @brief 将点转换为点云
 * @param points 输入的点
 * @return 返回点云 
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr
cloud::pointsToCloud(const std::vector<pcl::PointXYZ> &points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto &point: points) {
        cloud->push_back(point);
    }

    return cloud;
}


/**
 * @brief 将点云的X轴都压缩到0
 * @param cloud_in 输入的点云
 * @return 返回压缩后的点云 
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr
cloud::compressX(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in) {
    // 复制点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_compressed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_in, *cloud_compressed);

    // 将X轴都压缩到0
    for (auto &point: cloud_compressed->points) {
        point.x = 0;
    }
    return cloud_compressed;
}

/**
 * @brief 将点云的Y轴都压缩到0
 * @param cloud_in 输入的点云
 * @return 返回压缩后的点云 
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr
cloud::compressY(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in) {
    // 复制点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_compressed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_in, *cloud_compressed);

    // 将Y轴都压缩到0
    for (auto &point: cloud_compressed->points) {
        point.y = 0;
    }
    return cloud_compressed;
}


/**
 * 对点云进行聚类并返回所有的点云
 * @param cloud_in 
 * @param tolerance 
 * @param min_size 
 * @param max_size 
 * @return 
 */
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
cloud::euclideanCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float tolerance, int min_size,
                        int max_size) {

    // 
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_in);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setSearchMethod(tree);
    ec.setClusterTolerance(tolerance);
    ec.setMinClusterSize(min_size);
    ec.setMaxClusterSize(max_size);
    ec.setInputCloud(cloud_in);
    ec.extract(cluster_indices);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    for (const auto &indices: cluster_indices) 
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &index: indices.indices)
        {
            cloud_cluster->points.push_back(cloud_in->points[index]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }

    return clusters;
}

/**
 * @brief 对点云进行聚类，并返回最大的聚类点云数据
 * @param cloud_in 输入的点云 
 * @param tolerance 距离阈值 
 * @param min_size 最小的点数 
 * @param max_size 最大的点数 
 * @return 
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr
cloud::maxEuclideanCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float tolerance, int min_size,
                           int max_size) {
    auto clusters = euclideanCluster(cloud_in, tolerance, min_size, max_size);
    if (clusters.empty()) {
        return nullptr;
    }

    auto max_cluster = clusters[0];
    for (const auto &cluster: clusters) {
        if (cluster->size() > max_cluster->size()) {
            max_cluster = cluster;
        }
    }

    return max_cluster;
}


std::pair<bool, pcl::PointXYZ>
cloud::findNearestPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointXYZ &searchPoint) {
    // 如果点云为空，则返回一个无效的点
    if (cloud->empty()) {
        return {false, {}};
    }

    // 创建一个KdTree对象
    pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
    kd_tree.setInputCloud(cloud);

    // 最近邻搜索
    int K = 1;  // 返回最近的1个点
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    if (kd_tree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
        // 返回最近的点的坐标
        return {true, cloud->points[pointIdxNKNSearch[0]]};
    }

    // 如果没有找到最近的点，则返回一个无效的点
    return {false, {}};
}


/**
 * @brief 获取点云中Xy平面(Z取平均值)四个角上的点，并不是直接获取包围盒，而且或者实际存在点云上的四个角的点
 * @param cloud 输入的点云
 * @return 查找到的四个点
 */
std::vector<pcl::PointXYZ> cloud::findXyAnglePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    // 获取最大最小值
    auto [min_point, max_point] = getMinMax(cloud);

    // 统计点云获取平均Z值
    float z = 0.0;
    for (auto &point: cloud->points) {
        z += point.z;
    }
    z = z / cloud->size();

    // 根据最大最小和平均Z值生成初始化的四个角点
    auto p1 = pcl::PointXYZ{min_point.x, min_point.y, z};
    auto p2 = pcl::PointXYZ{min_point.x, max_point.y, z};
    auto p3 = pcl::PointXYZ{max_point.x, min_point.y, z};
    auto p4 = pcl::PointXYZ{max_point.x, max_point.y, z};

    // 查找最近的点
    auto [r1, p1_new] = findNearestPoint(cloud, p1);
    auto [r2, p2_new] = findNearestPoint(cloud, p2);
    auto [r3, p3_new] = findNearestPoint(cloud, p3);
    auto [r4, p4_new] = findNearestPoint(cloud, p4);

    // 保留x、y，z取平均值
    p1_new.z = z;
    p2_new.z = z;
    p3_new.z = z;
    p4_new.z = z;

    return {p1_new, p2_new, p3_new, p4_new};
}


/**
 * @brief 对点云进行平面拟合
 * @param cloud_in 
 * @param tolerance 
 * @param min_size 
 * @param max_size 
 * @return 
 */
pcl::ModelCoefficients::Ptr
cloud::fitPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float distance_threshold) {
    // 对点云进行拟合
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distance_threshold);
    seg.setInputCloud(cloud_in);
    seg.segment(*indices, *coefficients);
    return coefficients;
}


/**
 * @brief 对点云进行直线拟合
 * @param cloud_in 输入点云
 * @param distance_threshold 距离阈值
 * @return 
 */
pcl::ModelCoefficients::Ptr
cloud::fitLine(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float distance_threshold) {
    // 对点云进行拟合
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);  // SAC_MLESAC
    seg.setDistanceThreshold(distance_threshold);
    seg.setInputCloud(cloud_in);
    seg.segment(*indices, *coefficients);
    return coefficients;
}


std::vector<std::pair<float, float>>
cloud::calcXMinZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float x_step) {
    std::unordered_map<float, float> x_min_z;
    auto [min_point, max_point] = getMinMax(cloud_in);

    // 直接遍历所有的点
    for (auto &point: cloud_in->points) {
        // 获取对应的X的范围
        auto x_index = (int) ((point.x - min_point.x) / x_step);
        auto x = min_point.x + x_index * x_step;

        // 判断是否存在
        if (x_min_z.find(x) == x_min_z.end() || point.z < x_min_z[x]) {
            x_min_z[x] = point.z;
        }
    }

    // 转换成vector, 并按照x排序
    std::vector<std::pair<float, float>> x_min_z_vec;
    x_min_z_vec.reserve(x_min_z.size());
    for (const auto &item: x_min_z) {
        x_min_z_vec.emplace_back(item);
    }

    std::sort(x_min_z_vec.begin(), x_min_z_vec.end(), [](const auto &a, const auto &b) {
        return a.first < b.first;
    });

    return x_min_z_vec;
}


std::pair<float, float> cloud::findMutationMinZX(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float x_step) {
    auto x_min_z = calcXMinZ(cloud_in, x_step); 
    
    // 查找最大的变化率的地方
    float mutation_rate = 0;
    float mutation_x = 0;
    float mutation_z_range = 0;
    for (int i = 1; i < x_min_z.size(); ++i) {
        auto [x1, z1] = x_min_z[i - 1];
        auto [x2, z2] = x_min_z[i];
        auto rate = std::abs(z2 - z1) / (x2 - x1);
        if (rate > mutation_rate) {
            mutation_rate = rate;
            mutation_x = (x1 + x2) / 2;
            mutation_z_range = std::abs(z2 - z1);
        }
    }
    
    std::cout << "=== 突变分析结果 ===" << std::endl;
    std::cout << "突变位置 X: " << mutation_x << std::endl;
    std::cout << "突变率: " << mutation_rate << std::endl;
    std::cout << "突变高度范围: " << mutation_z_range << std::endl;
    return {mutation_x, mutation_z_range};
}


pcl::PointCloud<pcl::PointXYZ>::Ptr plane::cropCloudByPlanDistance(const pcl::ModelCoefficients::Ptr &coefficients,
                                                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                                                                   float min_distance, float max_distance,
                                                                   bool is_signed) {
    // 平面点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);

    // 获取平面系数
    float sign = coefficients->values[2] < 0 ? -1 : 1;
    float A = sign * coefficients->values[0];
    float B = sign * coefficients->values[1];
    float C = sign * coefficients->values[2];
    float D = sign * coefficients->values[3];
    Eigen::Vector4f coff{A, B, C, D};

    // 将符合要求的点云数据存进cloud_out
    for (int i = 0; i < cloud_in->size(); ++i) {
        auto dis = is_signed ?
                   pointToPlaneDistanceSigned(cloud_in->points[i], coff) :
                   pointToPlaneDistance(cloud_in->points[i], coff);

        if (min_distance <= dis && dis <= max_distance) {
            plane->push_back(cloud_in->points[i]);
        }
    }

    return plane;
}


/**
 * @brief 根据XY以及平面方程计算Z
 * @param coefficients
 * @param x
 * @param y
 */
float plane::calcZ(const pcl::ModelCoefficients::Ptr &coefficients, float x, float y) {
    auto a = coefficients->values[0];
    auto b = coefficients->values[1];
    auto c = coefficients->values[2];
    auto d = coefficients->values[3];
    return (-a * x - b * y - d) / c;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr plane::cropCloudPlaneDown(const pcl::ModelCoefficients::Ptr& coefficients, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, float offset)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Vector4f plane_coeff(
        coefficients->values[0],
        coefficients->values[1],
        coefficients->values[2],
        coefficients->values[3]
    );

    float z_threshold = 0 + offset;
    for (const auto& point : cloud_in->points) {
        double distance = pcl::pointToPlaneDistanceSigned(point, plane_coeff);
        // 只保留距离 <= 0 的点（在平面及下方）
        if (distance <= z_threshold) {
            filtered_cloud->push_back(point);
        }
    }
    return filtered_cloud;
}

/**
 * @brief 计算点到直线的距离
 * @param line 直线参数 
 * @param point 点 
 * @return 
 */
double line::distance(const pcl::ModelCoefficients::Ptr &line, const pcl::PointXYZ &point) {
    // 点到直线的距离
    auto a = line->values[0];
    auto b = line->values[1];
    auto c = line->values[2];
    auto d = line->values[3];

    auto distance = std::abs(a * point.x + b * point.y + c * point.z + d) / std::sqrt(a * a + b * b + c * c);
    return distance;
}

/**
 * @brief 将角度值转换成-90到90范围内，比如179度应该转换成-1度
 * @param angle 
 * @return 
 */
double angle::clamp(double angle) {
    while (angle > 90) {
        angle -= 180;
    }
    while (angle < -90) {
        angle += 180;
    }
    return angle;
}