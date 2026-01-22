#include <numeric>
#include <pcl/common/io.h>
#include "train_scan.h"
#include "extensions.h"


TrainScan::TrainScan() : Scan(getDefaultConfig()) {
    chassis = TRAIN_CHASSIS;
}


 bool TrainScan::getCarriagesSingle(const pcl::PointCloud<pcl::PointXYZ>::Ptr& train_cloud,float x1,float y1,float x2,float y2, Carriage& output)
{
    try {

        auto car_border_cloud = cloud::cropCloudZ(train_cloud,
            config.car_height - config.train_top_border_crop_min_offset_z,
            config.car_height + config.train_top_border_crop_max_offset_z);

        auto car_crop_cloud = cloud::cropCloud(car_border_cloud,
            x1+500, x2-500,
            y1, y2,
            std::numeric_limits<float>::min(),
            std::numeric_limits<float>::max());


        auto min_max = calcCarriageBorder(car_crop_cloud, config);

        // 获取车厢的最小最大XY
        auto min_xy = std::make_pair(min_max.first.x, min_max.first.y);
        auto max_xy = std::make_pair(min_max.second.x, min_max.second.y);

        // 将范围往内缩小300mm
        auto min_xy_inner = std::make_pair(min_xy.first + config.train_inner_crop_offset,
            min_xy.second + config.train_inner_crop_offset);
        auto max_xy_inner = std::make_pair(max_xy.first - config.train_inner_crop_offset,
            max_xy.second - config.train_inner_crop_offset);

        // 裁剪车厢的点云
        auto car_crop_cloud_inner = cloud::cropCloud(train_cloud,
            min_xy_inner.first, max_xy_inner.first,
            min_xy_inner.second, max_xy_inner.second,
            std::numeric_limits<float>::min(),
            std::numeric_limits<float>::max());

        // 取[min_point.z, min_point.z + 500]的点云使得只保留接近车厢地面部分的点云，使得后续的平面拟合更加准确
        auto [min_point, max_point] = cloud::getMinMax(car_crop_cloud_inner);
        auto car_crop_cloud_inner_z = cloud::cropCloudZ(car_crop_cloud_inner, min_point.z, min_point.z + 500);

        // sor降噪
        car_crop_cloud_inner_z = cloud::sorFilter(car_crop_cloud_inner_z, config.saddle_sor_mean_k,
            config.saddle_sor_stddev_mul_thresh);
        saveCloudWhenDebug(car_crop_cloud_inner_z, "car_crop_cloud_inner_z_sor.pcd");

        // 平面拟合
        auto coefficients = cloud::fitPlane(car_crop_cloud_inner_z, 100);
        // 如果拟合失败则不处理 
        if (coefficients->values.empty()) {
            saveCloudToPcd(car_crop_cloud_inner_z, "car_crop_cloud_inner_z.pcd");
            throw std::runtime_error("拟合平面失败");
        }

        // TODO: 可能还需要处理
        auto border = PlaneQuadrilateral{
                pcl::PointXYZ(min_max.first.x, min_max.first.y, min_max.first.z),
                pcl::PointXYZ(min_max.second.x, min_max.first.y, min_max.first.z),
                pcl::PointXYZ(min_max.second.x, min_max.second.y, min_max.first.z),
                pcl::PointXYZ(min_max.first.x, min_max.second.y, min_max.first.z) };

        output = Carriage({ border, coefficients, car_crop_cloud_inner});
        return true;
        }
        catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
            return false;
        }
}

 Carriage TrainScan::getCarriage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& car_cloud,
     const pcl::PointCloud<pcl::PointXYZ>::Ptr& carriage_cloud)
 {
     return Carriage();
 }
 
 /**
 * @brief 从整体的点云中提取出车厢信息，包含车厢部分的点云（一般不包含围栏），车厢的边界信息
 * @param cloud_in 
 * @return 
 */
std::vector<Carriage> TrainScan::getCarriages(const pcl::PointCloud<pcl::PointXYZ>::Ptr &train_cloud) {
    std::cout << "使用TrainScan::getCarriages检测" << std::endl;
    // 根据Z高度裁剪指定范围的，范围为车厢的高度-500mm到车厢的高度+500mm

    auto car_border_cloud = cloud::cropCloudZ(train_cloud,
                                              config.car_height - config.train_top_border_crop_min_offset_z,
                                              config.car_height + config.train_top_border_crop_max_offset_z);

    // 进行聚类: 用于分离不同的车厢
    auto car_border_clusters = cloud::euclideanCluster(car_border_cloud, config.carriage_cluster_tolerance,
                                                       config.carriage_cluster_min_size);
    saveCloudWhenDebug(car_border_cloud, "car_border_cloud.pcd");

    // 根据基本信息过滤掉不符合的点云
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> car_border_clouds;
    auto index = 0;
    for (auto &cluster: car_border_clusters) {
        // 获取大小
        auto [min_point, max_point] = cloud::getMinMax(cluster);
        auto [size_x, size_y, size_z] = cloud::getXyzSize(min_point, max_point);
        saveCloudWhenDebug(cluster, "cluster_" + std::to_string(index) + "_"  + std::to_string(size_x) + "_" + std::to_string(size_y) + ".pcd");

        // 判断大小: 判断Y方向，因为X方向可能只扫描到半截
        if ((size_y > config.car_width * 0.9) &&  // 车厢宽度满足要求
            (max_point.z > config.car_height * 0.8) && // 车厢高度满足要求
            isFullCarriage(cluster, config)) { // 是否是完整的车厢
            car_border_clouds.push_back(cluster);
        }
        
        index++;
    }

    // 获取车厢的范围
    std::vector<Carriage> car_borders;
    for (auto &one_car_border_cloud: car_border_clouds) {
        try {
            auto min_max = calcCarriageBorder(one_car_border_cloud, config);

            // 获取车厢的最小最大XY
            auto min_xy = std::make_pair(min_max.first.x, min_max.first.y);
            auto max_xy = std::make_pair(min_max.second.x, min_max.second.y);

            // 将范围往内缩小300mm
            auto min_xy_inner = std::make_pair(min_xy.first + config.train_inner_crop_offset,
                                               min_xy.second + config.train_inner_crop_offset);
            auto max_xy_inner = std::make_pair(max_xy.first - config.train_inner_crop_offset,
                                               max_xy.second - config.train_inner_crop_offset);

            // 裁剪车厢的点云
            auto car_crop_cloud_inner = cloud::cropCloud(train_cloud,
                                                         min_xy_inner.first, max_xy_inner.first,
                                                         min_xy_inner.second, max_xy_inner.second,
                                                         std::numeric_limits<float>::min(),
                                                         std::numeric_limits<float>::max());

            // 取[min_point.z, min_point.z + 500]的点云使得只保留接近车厢地面部分的点云，使得后续的平面拟合更加准确
            auto [min_point, max_point] = cloud::getMinMax(car_crop_cloud_inner);
            auto car_crop_cloud_inner_z = cloud::cropCloudZ(car_crop_cloud_inner, min_point.z, min_point.z + 500);

            // sor降噪
            car_crop_cloud_inner_z = cloud::sorFilter(car_crop_cloud_inner_z, config.saddle_sor_mean_k,
                                                      config.saddle_sor_stddev_mul_thresh);
            saveCloudWhenDebug(car_crop_cloud_inner_z, "car_crop_cloud_inner_z_sor.pcd");

            // 平面拟合
            auto coefficients = cloud::fitPlane(car_crop_cloud_inner_z, 100);
            // 如果拟合失败则不处理 
            if (coefficients->values.empty()) {
                saveCloudToPcd(car_crop_cloud_inner_z, "car_crop_cloud_inner_z.pcd");
                throw std::runtime_error("拟合平面失败");
            }

            // TODO: 可能还需要处理
            auto border = PlaneQuadrilateral{
                    pcl::PointXYZ(min_max.first.x, min_max.first.y, min_max.first.z),
                    pcl::PointXYZ(min_max.second.x, min_max.first.y, min_max.first.z),
                    pcl::PointXYZ(min_max.second.x, min_max.second.y, min_max.first.z),
                    pcl::PointXYZ(min_max.first.x, min_max.second.y, min_max.first.z)
            };

            // 火车的平面方程不在这里提取
            car_borders.push_back({border, coefficients, car_crop_cloud_inner});
        }
        catch (const std::exception &e) {
            std::cerr << e.what() << std::endl;
        }
    }

    return car_borders;
}

/**
 * 判断是否是完整的车厢
 * @param car_border_cloud 单个车厢边框上部点云
 * @param scan_config 
 * @return 
 */
bool
TrainScan::isFullCarriage(const pcl::PointCloud<pcl::PointXYZ>::Ptr &car_border_cloud, const ScanConfig &scan_config) {
    // 获取最大最小
    auto [min_point, max_point] = cloud::getMinMax(car_border_cloud);

    // 获取平均Y，并且裁剪其上下500mm范围的点云: 目的是剔除长边，只保留短边，如果车厢完整短边应该有两条
    auto average_y = (min_point.y + max_point.y) / 2;
    auto car_body_cloud = cloud::cropCloudY(car_border_cloud, average_y - 500, average_y + 500);

    // 聚类裁剪后的点云: 用于判断是否有两条短边
    auto car_body_clusters = cloud::euclideanCluster(car_body_cloud, 100, 10);

    // 判断是否有两条短边: 由于点云扫描的时候有部分断开，所以聚类可能大于2，所以只要不是1就可以
    return car_body_clusters.size() > 1;
}


/**
 * 计算车厢边界
 * @param car_border_cloud 输入的点云
 * @param scan_config 扫描配置 
 * @return 
 */
MinMaxPoint
TrainScan::calcCarriageBorder(const pcl::PointCloud<pcl::PointXYZ>::Ptr &car_border_cloud,
                              const ScanConfig &scan_config) 
{
    // 获取最大最小
    auto [min_point, max_point] = cloud::getMinMax(car_border_cloud);

    // 获取中心点
    auto center = new pcl::PointXYZ((min_point.x + max_point.x) / 2,
                                    (min_point.y + max_point.y) / 2,
                                    (min_point.z + max_point.z) / 2);

    // 根据Z裁剪，使得减小下面的冗余点的影响
    auto car_border_cloud_z = cloud::cropCloudZ(car_border_cloud, max_point.z - 200, max_point.z);

    // 基于中心点Y扩展500mm，然后分别以中心点X为中心，各裁剪出两部分点云
    auto car_border_cloud_x_a = cloud::cropCloud(car_border_cloud_z,
                                                 std::numeric_limits<float>::min(), center->x,
                                                 center->y - 500, center->y + 500,
                                                 std::numeric_limits<float>::min(), std::numeric_limits<float>::max());
    if (car_border_cloud_x_a->size() <= 0)
        return std::make_pair(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 0, 0));
    auto car_border_cloud_x_b = cloud::cropCloud(car_border_cloud_z,
                                                 center->x, std::numeric_limits<float>::max(),
                                                 center->y - 500, center->y + 500,
                                                 std::numeric_limits<float>::min(), std::numeric_limits<float>::max());

    if (car_border_cloud_x_b->size() <= 0)
        return std::make_pair(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 0, 0));
    // 基于中心点X扩展5000，然后分别以中心点Y为中心，各裁剪出两部分点云
    auto car_border_cloud_y_a = cloud::cropCloud(car_border_cloud_z,
                                                 center->x - 500, center->x + 500,
                                                 std::numeric_limits<float>::min(), center->y,
                                                 std::numeric_limits<float>::min(), std::numeric_limits<float>::max());

    if (car_border_cloud_y_a->size() <= 0)
        return std::make_pair(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 0, 0));
    auto car_border_cloud_y_b = cloud::cropCloud(car_border_cloud_z,
                                                 center->x - 500, center->x + 500,
                                                 center->y, std::numeric_limits<float>::max(),
                                                 std::numeric_limits<float>::min(), std::numeric_limits<float>::max());
    if (car_border_cloud_y_b->size() <= 0)
        return std::make_pair(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 0, 0));

    // 分别计算两个x的平均x值
    auto mean_x_a = cloud::getMean(car_border_cloud_x_a).x;
    auto mean_x_b = cloud::getMean(car_border_cloud_x_b).x;

    // 分别计算两个y的平均y值
    auto mean_y_a = cloud::getMean(car_border_cloud_y_a).y;
    auto mean_y_b = cloud::getMean(car_border_cloud_y_b).y;

    // 获取最小最大XY
    auto [min_x, max_x] = std::minmax(mean_x_a, mean_x_b);
    auto [min_y, max_y] = std::minmax(mean_y_a, mean_y_b);

    return std::make_pair(pcl::PointXYZ(min_x, min_y, min_point.z), pcl::PointXYZ(max_x, max_y, max_point.z));
}


/**
 * @brief 格式化输出点云, 火车的输出点云需要去掉车厢壁
 * @param cloud: 输入点云
 * @param cloud_out: 输出用的点云
 * @param results: 识别的结果，用于辅助裁剪
 */
void TrainScan::formatOutputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                  pcl::PointCloud<pcl::PointXYZ> *cloud_out,
                                  const std::vector<DetectResult> &results) {

    // 获取所有的车厢的结果（data_type = 3\4）
    auto carriage_min_results = new std::vector<DetectResult>();
    auto carriage_max_results = new std::vector<DetectResult>();
    for (auto &result: results) {
        if (result.data_type == 3) {
            carriage_min_results->push_back(result);
        }
        if (result.data_type == 4) {
            carriage_max_results->push_back(result);
        }
    }
    
    // 如果车厢结果为空，则返回空点云
    if (carriage_min_results->empty() || carriage_max_results->empty()) {
        return;
    }

    // 获取所有min的平均值(结果中的Y值)
    auto mean_min_y = std::accumulate(carriage_min_results->begin(), carriage_min_results->end(), 0.0,
                                      [](double sum, const DetectResult &result) {
                                          return sum + result.y;
                                      }) / static_cast<double>(carriage_min_results->size());
    auto mean_max_y = std::accumulate(carriage_max_results->begin(), carriage_max_results->end(), 0.0,
                                      [](double sum, const DetectResult &result) {
                                          return sum + result.y;
                                      }) / static_cast<double>(carriage_max_results->size());

    auto min_x = std::min_element(carriage_min_results->begin(), carriage_min_results->end(),
                                  [](const DetectResult &a, const DetectResult &b) {
                                      return a.x < b.x;
                                  })->x;
    auto max_x = std::max_element(carriage_max_results->begin(), carriage_max_results->end(),
                                  [](const DetectResult &a, const DetectResult &b) {
                                      return a.x < b.x;
                                  })->x;

    // 裁剪车厢的点云
    auto crop_out_cloud = cloud::cropCloud(cloud,
                                           static_cast<float>(min_x + 500), static_cast<float>(max_x + 500),
                                           static_cast<float>(mean_min_y + 200), static_cast<float>(mean_max_y - 200),
                                           std::numeric_limits<float>::min(),
                                           std::numeric_limits<float>::max());
    saveCloudWhenDebug(crop_out_cloud, "crop_out_cloud.pcd");

    pcl::copyPointCloud(*crop_out_cloud, *cloud_out);
}
