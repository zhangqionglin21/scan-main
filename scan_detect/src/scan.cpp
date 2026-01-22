#include "scan.h"
#include "extensions.h"
#include "../../cylinder_fitting/include/vec3.h"
#include "../../cylinder_fitting/include/export.h"
#include "../../cylinder_fitting/include/algorithm.h"
#include "pattern.h"

#include <iostream>
#include <utility>

#include <boost/filesystem/directory.hpp>
#include <boost/filesystem/operations.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>


/**
 * @brief 通过配置构造
 * @param default_config 
 */
Scan::Scan(ScanConfig default_config) {
    this->config = default_config;

//    loadConfig(default_config_path);
}

/**
 * @brief 指定配置文件构造
 * @param path 配置文件路径 
 */
Scan::Scan(const std::string &path) {
    loadConfig(path);
}

/**
 * @brief 获取默认配置
 * @return 
 */
ScanConfig Scan::getDefaultConfig() {
    return {};
}

/**
 * @brief 从配置文件中加载配置并进行检测
 * @param path 配置文件路径
 */
std::vector<DetectResult> Scan::detectFromFile(const std::string &path) {
    // 判断文件是否存在
    if (!boost::filesystem::exists(path)) {
        throw std::runtime_error("文件不存在: " + path);
    }

    // 读取点云文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud);

    return detect(cloud, nullptr);
}

/**
 * 检测
 * @return 
 */
std::vector<DetectResult>
Scan::detect(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ> *cloud_out) {
    // 保存输入点云
    saveCloudWhenDebug(cloud, "01-输入点云.pcd");
    std::cout << "保存输输入点云完成" << std::endl;

    // 统一点云单位为mm
    auto [area_x_size, area_y_size, area_z_size] = config.areaSize();
    auto scaled_cloud = autoScaleCloud(cloud, area_x_size, area_y_size, area_z_size);
    std::cout << "自动缩放点云完成" << std::endl;

    // 检查是否需要交换XY坐标（X应为大坐标）
    int switch_flag = -1;
    auto switched_cloud = trySwitchCloudXy(scaled_cloud, switch_flag);
    saveCloudWhenDebug(switched_cloud, "02-XY交换后点云.pcd");
    std::cout << "转换坐标系完成" << std::endl;

    // 裁剪点云：去除地面及车辆四周影响因素
    auto cropped_cloud = cleanPointCloud(switched_cloud);
    saveCloudWhenDebug(cropped_cloud, "03-自动裁剪后点云.pcd");
    std::cout << "裁剪点云完成: " << cropped_cloud->size() << std::endl;

    // 获取车厢点云：根据配置文件车厢长宽高筛选聚类结果
    auto carriage_clouds = getCarriageClouds(cropped_cloud);
    std::cout << "获取车厢点云完成: " << carriage_clouds.size() << std::endl;

    // 获取车厢信息：计算车厢底部平面、车厢角点、车厢点云
    auto carriages = std::vector<Carriage>();
    for (auto& carriage_cloud : carriage_clouds)
        carriages.push_back(getCarriage(cropped_cloud, carriage_cloud));
    std::cout << "获取车厢信息完成: " << carriages.size() << std::endl;

    // 对车厢进行排序
    auto sorted_carriages = sortCarriages(carriages);
    std::cout << "车厢排序完成" << std::endl;

    //这里做个判断，如果两个车厢的间距超过平均间距的5倍或以上，则认为丢了不该丢的车厢点
    if(sorted_carriages.size() > 1)
    {
        for (auto it = sorted_carriages.begin(); it != sorted_carriages.end() - 1;)
        {
            auto carriage1 = *(it);
            auto carriage2 = *(it+1);
            float center1 = (carriage1.border.min_max.x + carriage1.border.max_min.x) / 2;
            float center2 = (carriage2.border.min_min.x + carriage2.border.max_max.x) / 2;

            float tempDis = center2 - center1;

            if (tempDis > 3000.0f) //间距超过3m这里就可能要插入
            {
                //获取点云
                Carriage crriagesingle;
                if(getCarriagesSingle(switched_cloud, carriage1.border.min_max.x, carriage1.border.min_max.y, carriage2.border.max_max.x, carriage2.border.max_max.y, crriagesingle))
                    it = sorted_carriages.insert(it+1, crriagesingle);
                else
                    ++it;
            }
            else
                ++it;
        }
    }

    // 遍历车厢进行识别
    auto all_detected_results = std::vector<DetectResult>();
    for (auto i = 0; i < sorted_carriages.size(); i++) {
        // 判断车厢类型
        auto carriage = sorted_carriages[i];
        auto [carriage_type, carriage_angle] = getCarriagesType(carriage.cloud, switch_flag);
        std::string type_str = carriage_type == FLATBED_TRUCK? "平板车厢" : "高栏车厢";
        std::cout << "获取车厢类型完成: " << type_str << std::endl;
        if (carriage_type == FLATBED_TRUCK)
        {
            saveCloudWhenDebug(carriage.cloud, "07-" + std::to_string(i) + "号平板车厢点云.pcd");

            // 获取钢卷和鞍座的点云
            auto [saddles_cloud, coils_cloud] = getSaddleAndCoilCloud(carriage);

            // 识别钢卷
            auto coils = detectCoils(coils_cloud);
            std::cout << "钢卷识别结果: " << std::endl;
            for (auto& coil : coils)
            {
                std::cout<<"    X, Y, Z: "<<std::to_string(coil.x)<<", "<<std::to_string(coil.y)<<", "<<std::to_string(coil.z)<<std::endl;
                std::cout << "    宽度: " << std::to_string(coil.width)<< std::endl;
            }

            // 识别鞍座
            auto saddles = detectSaddles(saddles_cloud, coils, carriage);
            std::cout << "鞍座识别结果: " << std::endl;
            for (auto& saddle : saddles)
            {
                std::cout << "    X, Y, Z: " << std::to_string(saddle.x) << ", " << std::to_string(saddle.y) << ", " << std::to_string(saddle.z) << std::endl;
                std::cout << "    宽度: " << std::to_string(saddle.width) << std::endl;
            }
            for (auto& result : saddles)
            {
                result.axis = carriage_angle;
            }

            // 合并结果
            auto car_index = i + 1;
            auto merge_results = mergeDetectResult(coils_cloud, coils, saddles_cloud, saddles, carriage.border.getMinMax(),
                car_index);

            // 添加结果
            all_detected_results.insert(all_detected_results.end(), merge_results.begin(), merge_results.end());
        }
        else
        {
            saveCloudWhenDebug(carriage.cloud, "07-" + std::to_string(i) + "号高栏车厢点云.pcd");

            // 去除车厢边框
            // 定义原始边界点
            std::vector<Eigen::Vector3f> original_boundary = {
                Eigen::Vector3f(carriage.border.min_min.x, carriage.border.min_min.y, 0.0f),
                Eigen::Vector3f(carriage.border.max_min.x, carriage.border.max_min.y, 0.0f),
                Eigen::Vector3f(carriage.border.max_max.x, carriage.border.max_max.y, 0.0f),
                Eigen::Vector3f(carriage.border.min_max.x, carriage.border.min_max.y, 0.0f)
            };

            // 计算内缩后的边界点
            std::vector<Eigen::Vector3f> shrunk_boundary = shrinkQuadrilateral(original_boundary, config.fence_shrink_distance);

            // 裁剪点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud = cropPointCloudWithQuadrilateral(carriage.cloud, shrunk_boundary);
            std::cout << "裁剪围栏后车厢点云大小: " << cropped_cloud->size() << std::endl;
            saveCloudWhenDebug(cropped_cloud, "08-" + std::to_string(i) + "号高栏车厢去除围栏后点云.pcd");

            // 获取车厢内部点云信息
            auto [min_point, max_point] = cloud::getMinMax(cropped_cloud);

            // 获取所有钢卷点云
            auto coils_cloud = cloud::cropCloud(cropped_cloud,
                min_point.x, max_point.x,
                min_point.y, max_point.y,
                min_point.z + 1000.0f, max_point.z);
            saveCloudWhenDebug(coils_cloud, "09-" + std::to_string(i) + "车厢钢卷点云.pcd");

            // 识别钢卷
            auto coils = detectCoils(coils_cloud);
            std::cout << "钢卷识别结果: " << std::endl;
            for (auto& coil : coils)
            {
                std::cout << "    X, Y, Z: " << std::to_string(coil.x) << ", " << std::to_string(coil.y) << ", " << std::to_string(coil.z) << std::endl;
                std::cout << "    宽度: " << std::to_string(coil.width) << "   直径: " << std::to_string(coil.diameter) << std::endl;
            }

            // 获取所有鞍座点云
            auto crop_saddles_cloud = cloud::cropCloud(cropped_cloud,
                min_point.x, max_point.x,
                min_point.y, max_point.y,
                min_point.z, min_point.z + config.highsided_bottom_z_range);
            auto first_coefficients = cloud::fitPlane(crop_saddles_cloud, 100);
            auto cloud_plane_up = plane::cropCloudByPlanDistance(first_coefficients, crop_saddles_cloud, 0, 40);
            saveCloudWhenDebug(cloud_plane_up, "10-" + std::to_string(i) + "-车厢底部平面拟合筛选后点云.pcd");
            auto saddles_cloud = getPointCloudDifference(crop_saddles_cloud, cloud_plane_up, 0.01f);
            saveCloudWhenDebug(saddles_cloud, "11-" + std::to_string(i) + "车厢底部所有鞍座点云.pcd");

            // 识别鞍座
            auto saddles = detectHighsidedSaddles(saddles_cloud, coils);
            std::cout << "鞍座识别结果数量: " << saddles.size() << std::endl;
            for (auto& saddle : saddles)
            {
                std::cout << "    X, Y, Z: " << std::to_string(saddle.x) << ", " << std::to_string(saddle.y) << ", " << std::to_string(saddle.z) << std::endl;
                std::cout << "    宽度: " << std::to_string(saddle.width) << "   直径: " << std::to_string(saddle.diameter) << std::endl;
            }

            // 合并结果
            auto car_index = i + 1;
            auto merge_results = mergeDetectResult(coils_cloud, coils, saddles_cloud, saddles, carriage.border.getMinMax(),
                car_index);

            // 添加结果
            all_detected_results.insert(all_detected_results.end(), merge_results.begin(), merge_results.end());
        }
        for (auto& result : all_detected_results)
        {
            if (result.data_type == DetectType::CAR_HEAD_POINT || result.data_type == DetectType::CAR_TAIL_POINT)
                result.axis = carriage_angle;
        }
    }
    std::cout << "识别后结果数量: " << all_detected_results.size() << std::endl;

    // 格式化结果 
    auto format_results = formatDetectResult(all_detected_results);
    std::cout << "格式化结果数量: " << format_results.size() << std::endl;

    // 格式化结果: 如果有异常会直接抛出异常
    checkDetectResults(format_results);
    saveCloudWhenDebug(switchResultToCloud(format_results), "result_points.pcd");
    std::cout << "检查后结果数量: " << format_results.size() << std::endl;
    
    // 转换结果XY坐标
    auto switched_results = trySwitchResultXy(cropped_cloud, format_results, switch_flag);
    std::cout << "结果XY坐标转换完成" << std::endl;

    // 复制输出点云
    if (cloud_out != nullptr) {
        formatOutputCloud(cropped_cloud, cloud_out, switched_results);
    }
    std::cout << "复制输出点云完成" << std::endl;

    std::cout << "Result count: " << switched_results.size() << std::endl;
    return switched_results;
}


/**
 * @brief 对车厢结果排序，按照x轴从小到大排序
 * @param carriages 车厢信息
 * @return 排序后的车厢信息 
 */
std::vector<Carriage> Scan::sortCarriages(const std::vector<Carriage> &carriages) {
    auto sorted_carriages = carriages;
    std::sort(sorted_carriages.begin(), sorted_carriages.end(), [](const Carriage &a, const Carriage &b) {
        return a.border.min_min.x < b.border.min_min.x;
    });
    return sorted_carriages;
}


/**
 * @brief 从整体或者单个的点云中提取鞍座和钢卷的点云
 * @param carriage 车厢范围以及点云信息
 * @return 
 */
std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr>
Scan::getSaddleAndCoilCloud(const Carriage &carriage) {
    // 获取鞍座点云
    auto saddles_cloud = getSaddleCloud(carriage);
    saveCloudWhenDebug(saddles_cloud, "08-00鞍座点云.pcd");

    // 获取钢卷点云
    auto coils_cloud = getCoilCloud(carriage);
    saveCloudWhenDebug(coils_cloud, "08-01钢卷点云.pcd");

    return {saddles_cloud, coils_cloud};
}

/**
 * @brief 优先使用默认配置，如果路径下不存在配置文件，则自动生成默认配置文件，如果有配置文件，则加载配置文件，支持只设置部分配置
 * @param path 配置文件路径
 */
bool Scan::loadConfig(const std::string &path) {
    try {
        // 判断路径是否存在，如果不存在，将当前的默认配置保存到文件中
        if (!boost::filesystem::exists(path)) {
            config.saveConfigToJson(path);
        }

        // 加载配置
        config.loadConfigFromJson(path);

        return true;
    } catch (std::exception &e) {
        std::cerr << "加载配置文件失败: " << e.what() << "使用默认配置" << std::endl;
        return false;
    }
}

/**
 * @brief 对整体点云进行预处理，主要进行范围的裁剪，剔除地面等 
 * @param cloud_in 原始点云
 * @param config 配置信息
 * @return 预处理后的点云 
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr Scan::cleanPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in) {
    if (config.cloud_auto_crop) {
        // 下采样
        auto down_sampled_cloud = cloud::downSample(cloud_in, 100);
        // 移除离群点
        auto removed_outliers_cloud = cloud::sorFilter(down_sampled_cloud, 100, 2.0);
        // 获取范围。
        auto [min_point, max_point] = cloud::getMinMax(removed_outliers_cloud);
        std::cout << "点云范围: X[" << min_point.x << ", " << max_point.x << "] Y[" << min_point.y << ", " << max_point.y << "] Z[" << min_point.z << ", " << max_point.z << "]" << std::endl;
        // 缩小范围: X缩小1000， Y缩小500， Z缩小500
//        auto cropped_cloud = cloud::cropCloud(cloud_in,
//                                              min_point.x + 1000, max_point.x - 1000,
////                                              min_point.y + 500, max_point.y - 500, // 不裁剪y方向，避免将倾斜的车厢裁剪了
//                                              min_point.y, max_point.y,
//                                              min_point.z + 500,
//                                              config.cloud_crop_z_max); // TODO: 临时处理，因为sor的时候扫描头上方的干扰点没有去掉，先人工强制去除
        auto cropped_cloud = cloud::cropCloud(cloud_in,
            min_point.x + 200, max_point.x - 200,
            min_point.y, max_point.y, // 不裁剪y方向，避免将倾斜的车厢裁剪了
            min_point.z + 1500,
            config.cloud_crop_z_max);

        return cropped_cloud;

    } else {
        // 按照区域的范围先进行裁剪
        auto cropped_cloud = cloud::cropCloud(cloud_in,
                                              config.cloud_crop_x_min, config.cloud_crop_x_max,
                                              config.cloud_crop_y_min, config.cloud_crop_y_max,
                                              config.cloud_crop_z_min, config.cloud_crop_z_max);
        return cropped_cloud;
    }
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Scan::getCarriageClouds(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in)
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> carriage_clouds;
    auto cluster_clouds = cloud::euclideanCluster(cloud_in, 200, 10);
    for (auto& cluster_cloud : cluster_clouds)
    {
        try
        {
            // 获取聚类点云的尺寸
            auto [min_point, max_point] = cloud::getMinMax(cluster_cloud);
            auto [x_size, y_size, z_size] = cloud::getXyzSize(min_point, max_point);
            // 判断是否在车身长度、宽度的范围内
            if (x_size > (config.car_length * 0.8) && y_size > (config.car_width * 0.8) &&
                max_point.z > (config.car_height * 0.8))
            {
                carriage_clouds.push_back(cluster_cloud);
            }
        }
        catch (std::exception& e)
        {
            std::cout << "异常是: " << e.what() << std::endl;
        }
    }
    return carriage_clouds;
}

/**
* @brief 获取车辆类型
* @param clouds_in 输入的点云
* @return 车辆类型: 0-平板货车，1-高栏式货车
*/
std::pair<int, float> Scan::getCarriagesType(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, int switch_flag)
{
    // 为空采用默认检测
    if (cloud_in->empty())
        return { FLATBED_TRUCK, 0.0f };

    // 获取点云信息
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud_in, min_pt, max_pt);

    // 提取头部点云（前10%）
    pcl::PointCloud<pcl::PointXYZ>::Ptr head_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> head_pass;
    head_pass.setInputCloud(cloud_in);
    head_pass.setFilterFieldName("x");
    head_pass.setFilterLimits(min_pt.x, min_pt.x + config.head_tail_cut_range);
    head_pass.filter(*head_cloud);

    // 提取尾部点云（后10%）
    pcl::PointCloud<pcl::PointXYZ>::Ptr tail_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> tail_pass;
    tail_pass.setInputCloud(cloud_in);
    tail_pass.setFilterFieldName("x");
    tail_pass.setFilterLimits(max_pt.x - config.head_tail_cut_range, max_pt.x);
    tail_pass.filter(*tail_cloud);

    // 获取中间点云
    // 提取中间80%点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr middle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> middle_pass;
    middle_pass.setInputCloud(cloud_in);
    middle_pass.setFilterFieldName("x");
    middle_pass.setFilterLimits(
        min_pt.x + config.head_tail_cut_range,
        max_pt.x - config.head_tail_cut_range
    );
    middle_pass.filter(*middle_cloud);

    float car_axis = computeMinBoundingBoxAnglePCA(middle_cloud);

    // 去除头尾孤立点
    // head_cloud = cloud::sorFilter(head_cloud, 50, 1.2);
    // tail_cloud = cloud::sorFilter(tail_cloud, 50, 1.2);

    saveCloudWhenDebug(head_cloud, "07-00头部区域点云.pcd");
    saveCloudWhenDebug(tail_cloud, "07-01尾部区域点云.pcd");

    // 计算区域高度
    pcl::getMinMax3D(*head_cloud, min_pt, max_pt);
    auto head_height = max_pt.z - min_pt.z;
    pcl::getMinMax3D(*tail_cloud, min_pt, max_pt);
    auto tail_height = max_pt.z - min_pt.z;

    auto height_diff = std::abs(head_height - tail_height);

    std::cout << "头部区域高度: " << head_height << "mm" << std::endl;
    std::cout << "尾部区域高度: " << tail_height << "mm" << std::endl;
    std::cout << "头尾高度差: " << height_diff << "mm" << std::endl;

    if (head_height > config.highsided_height_min && tail_height > config.highsided_height_min)
        return { HIGHSIDED_TRUCK, car_axis };
    else
        return { FLATBED_TRUCK, car_axis };
}

/**
* @brief 计算点云平均高度
* @param cloud 输入点云
* @return 点云平均高度
*/
float Scan::calculateAverageHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    if (cloud->empty()) return 0.0f;

    float sum = 0.0f;
    for (const auto& point : cloud->points) {
        sum += point.z;
    }
    return sum / cloud->size();
}


/**
 * 提取鞍座点云
 * @param cloud_in 输入的车厢点云，包含鞍座和钢卷
 * @param saddle_cloud 提取的鞍座的点云（鞍座整体的点云） 
 * @param coefficients 车厢平面的方程 
 * @return 
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr Scan::getSaddleCloud(const Carriage &carriage) {
    // 获取范围
    auto [min_point, max_point] = carriage.border.getMinMax();

    // 获取中心点
    auto center = carriage.getPlaneCenter();

    // 计算裁剪范围
    auto min_x = min_point.x + config.crop_saddle_offset_x;
    auto max_x = max_point.x - config.crop_saddle_offset_x;
    auto min_y = min_point.y + config.crop_saddle_offset_y;
    auto max_y = max_point.y - config.crop_saddle_offset_y;
//    auto min_z = center.z + config.crop_saddle_offset_z_min;
//    auto max_z = center.z + config.crop_saddle_offset_z_max;

    // 裁剪X、Y平面
    auto saddle_cloud = cloud::cropCloud(carriage.cloud, min_x, max_x, min_y, max_y, std::numeric_limits<float>::min(),
                                         std::numeric_limits<float>::max());

    // 基于拟合的平面根据距离裁剪
    saddle_cloud = plane::cropCloudByPlanDistance(carriage.plane, saddle_cloud, config.crop_saddle_offset_z_min,
                                                  config.crop_saddle_offset_z_max, true);

    // sor降噪
    auto filter_saddle_cloud = cloud::sorFilter(saddle_cloud,
                                                config.saddles_sor_mean_k,
                                                config.saddles_sor_stddev_mul_thresh);

    return saddle_cloud;
}

/**
 * 提取鞍座点云
 * @param cloud_in 输入的车厢点云，包含鞍座和钢卷
 * @param saddle_cloud 提取的鞍座的点云（鞍座整体的点云）
 * @param coefficients 车厢平面的方程
 * @return
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr Scan::getHighsidedSaddleCloud(const Carriage& carriage) {
    // 获取范围
    auto [min_point, max_point] = carriage.border.getMinMax();

    // 获取中心点
    auto center = carriage.getPlaneCenter();

    // 计算裁剪范围
    auto min_x = min_point.x + 300;
    auto max_x = max_point.x - 300;
    auto min_y = min_point.y + 300;
    auto max_y = max_point.y - 300;
    auto min_z = center.z + config.crop_saddle_offset_z_min;
    auto max_z = center.z + config.crop_saddle_offset_z_max;

        // 裁剪X、Y平面
    auto saddle_cloud = cloud::cropCloud(carriage.cloud, min_x, max_x, min_y, max_y, std::numeric_limits<float>::min(),
        std::numeric_limits<float>::max());

    // 基于拟合的平面根据距离裁剪
    saddle_cloud = plane::cropCloudByPlanDistance(carriage.plane, saddle_cloud, config.crop_saddle_offset_z_min,
        config.crop_saddle_offset_z_max, true);

    // sor降噪
    auto filter_saddle_cloud = cloud::sorFilter(saddle_cloud,
        config.saddles_sor_mean_k,
        config.saddles_sor_stddev_mul_thresh);

    return saddle_cloud;
}

/**
 * @brief 获取钢卷点云
 * @param cloud_in 
 * @param coil_cloud 
 * @param coefficients 
 * @param car_min_min 
 * @param car_min_max 
 * @param car_max_min 
 * @param car_max_max 
 * @param x_offset 
 * @param y_offset 
 * @return 
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr Scan::getCoilCloud(const Carriage &carriage) {
    // 获取范围
    auto [min_point, max_point] = carriage.border.getMinMax();

    // 获取中心点
    auto center = carriage.getPlaneCenter();

    // 计算裁剪范围
    auto min_x = min_point.x + config.crop_coil_offset_x;
    auto max_x = max_point.x - config.crop_coil_offset_x;
    auto min_y = min_point.y + config.crop_coil_offset_y;
    auto max_y = max_point.y - config.crop_coil_offset_y;
    auto min_z = center.z + config.crop_coil_offset_z_min;
    auto max_z = center.z + config.crop_coil_offset_z_max;

    // 裁剪鞍座的点云
    auto coil_cloud = cloud::cropCloud(carriage.cloud, min_x, max_x, min_y, max_y, min_z, max_z);
    if (coil_cloud->empty()) return coil_cloud;

    // sor降噪
    auto filter_coil_cloud = cloud::sorFilter(coil_cloud, config.coils_sor_mean_k, config.coils_sor_stddev_mul_thresh);

    return filter_coil_cloud;
}

/**
 * 识别钢卷
 * @param coils_cloud 包含所有钢卷的点云
 * @param config 配置信息
 * @return 识别的点云信息 
 */
std::vector<DetectResult>
Scan::detectCoils(const pcl::PointCloud<pcl::PointXYZ>::Ptr &coils_cloud) {
    // 如果没有点云则返回空
    if (coils_cloud->empty()) return {};

    // 获取所有的聚类的点云
    auto coils_clusters = getCoilClouds(coils_cloud);
    std::cout << "得到钢卷聚类: " << coils_clusters.size() << std::endl;

    // 遍历所有的聚类，尝试拟合钢卷
    std::vector<DetectResult> coils;
    for (auto &coil_cloud: coils_clusters) {
        // 对点云进行降噪sor
        auto sor_coil_cloud = cloud::sorFilter(coil_cloud, config.coil_sor_mean_k, config.coil_sor_stddev_mul_thresh);

        // 判断是否符合要求
        std::cout << "聚类是否为钢卷: " << isCoilCloud(sor_coil_cloud) << std::endl;
        if (!isCoilCloud(sor_coil_cloud)) continue;

        // 识别钢卷: TODO: 如果点云少，导致没有识别出钢卷，但是相同位置可能会识别出鞍座，按理不应该识别出鞍座，存在这种冲突情况的话应该取消对应的鞍座的识别
        auto detect_result = detectCoil(sor_coil_cloud);
        if (detect_result != nullptr) {
            // 添加结果
            coils.push_back(*detect_result);
        }
    }

    return coils;
}

/**
 * 获取单个钢卷的点云聚类，先尝试直接聚类，如果聚类出来的长度大于钢卷的最大直径，则将min_z + 100裁剪后再次聚类 
 */
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
Scan::getCoilClouds(const pcl::PointCloud<pcl::PointXYZ>::Ptr &coils_cloud) const {
    // 保存聚类的点云
    auto coil_clusters = new std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>();
    

    // 获取所有的聚类的点云
    auto coils_clusters = cloud::euclideanCluster(coils_cloud,
                                                  config.coil_cluster_tolerance,
                                                  config.coil_cluster_min_size);

    // 遍历所有的聚类，尝试看是否还需要再次拆分聚类
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> coils;
    int idx = 0;
    for (auto &coil_cloud: coils_clusters) {
        // 判断长度
        auto [min_point, max_point] = cloud::getMinMax(coil_cloud);
        auto size_x = std::abs(max_point.x - min_point.x);

        saveCloudWhenDebug(coil_cloud, "09-" + std::to_string(idx) + "号钢卷聚类点云.pcd");
        idx++;

        // 如果长度大于最大直径，则认为多个卷在一起，则剔除一部分底部，使得强制将钢卷分割开，然后再次聚类使得每个钢卷单独聚类
        if (size_x > config.coil_max_diameter) 
        {
            // sor降噪
            auto sor_coil_cloud = cloud::sorFilter(coil_cloud, config.coil_sor_mean_k, config.coil_sor_stddev_mul_thresh);
            
            // 裁剪min_z+100
            auto crop_coils_cloud = cloud::cropCloudZ(sor_coil_cloud, min_point.z + 200, max_point.z);
            if (crop_coils_cloud->empty()) continue;

            saveCloudWhenDebug(crop_coils_cloud, "09-" + std::to_string(idx) + "号钢卷聚类裁剪后点云.pcd");

            // 再次聚类
            for (auto i = 1; i < 5; i++){
                auto one_coil_clusters = cloud::euclideanCluster(crop_coils_cloud,
                                                                 config.coil_cluster_tolerance,
                                                                 config.coil_cluster_min_size);
                
                if (one_coil_clusters.size() > 1){
                    // 添加到结果中
                    for (auto &one_coil_cloud: one_coil_clusters) {
                        coil_clusters->push_back(one_coil_cloud);
                    }
                    break;
                }else {
                    crop_coils_cloud = cloud::cropCloudZ(sor_coil_cloud, min_point.z + 200 + (i + 1) * 100, max_point.z);
                    if (crop_coils_cloud->empty()) break;
                }
            }

        } 
        else 
        {
            coil_clusters->push_back(coil_cloud);
        }
    }
    
    return *coil_clusters;
}

/**
 * 识别单个钢卷
 * @param coil_cloud 单个钢卷的点云
 * @param config 配置信息 
 * @return 
 */
std::shared_ptr<DetectResult>
Scan::detectCoil(const pcl::PointCloud<pcl::PointXYZ>::Ptr &coil_cloud) {
    // 对点云进行降噪
    auto deNoise_coil_cloud = deNoiseCoilCloud(coil_cloud);
    if (deNoise_coil_cloud->empty()) return nullptr;

    // 拟合钢卷
    return std::make_shared<DetectResult>(fittingCoil(deNoise_coil_cloud, config));
}

/**
 * 对单个钢卷点云进行降噪
 * @param coil_cloud 单个钢卷的点云
 * @param scan_config 配置信息 
 * @return 
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr
Scan::deNoiseCoilCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &coil_cloud) {
    // 获取边界范围
    auto [min_point, max_point] = cloud::getMinMax(coil_cloud);

    // 裁剪侧面 TODO: 暂时只适用于钢卷轴心垂直于X轴的情况
    auto crop_min_y = min_point.y + config.coil_side_crop_length;
    auto crop_max_y = max_point.y - config.coil_side_crop_length;

    // 裁剪
    auto crop_coil_cloud = cloud::cropCloudY(coil_cloud, crop_min_y, crop_max_y);
    if (crop_coil_cloud->empty()) return crop_coil_cloud;

    // 查找最低点
    auto lowest_point = cloud::getMinZPoint(crop_coil_cloud);
    auto lowest_to_min_distance = abs(lowest_point.x - min_point.x);
    auto lowest_to_max_distance = abs(lowest_point.x - max_point.x);

    // 判断是否需要裁剪
    auto not_need_crop = lowest_to_min_distance > config.coil_axis_crop_threshold &&
                         lowest_to_max_distance > config.coil_axis_crop_threshold;
    if (not_need_crop) {
        return crop_coil_cloud;
    }

    auto is_near_min_x = lowest_to_min_distance < lowest_to_max_distance;
    if (is_near_min_x) {
        // 靠近最小X，说明这部分少，所以保留多的部分
        return cloud::cropCloudX(crop_coil_cloud, lowest_point.x + 10, max_point.x);
    } else {
        // 靠近最大X，说明这部分少，所以保留多的部分
        return cloud::cropCloudX(crop_coil_cloud, min_point.x, lowest_point.x - 10);
    }
}

/**
 * 判断点云是否是钢卷点云: 目前主要判断离车厢很近的且数量很少的点云
 * @param coil_cloud 
 * @param config 
 * @return 
 */
bool
Scan::isCoilCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &coil_cloud) {
    // 判断是否为空
    if (coil_cloud->empty()) return false;

    // 获取边界范围
    auto [size_x, size_y, size_z] = cloud::getCloudSize(coil_cloud);

    // 获取X、Y方向的大小: 如果太小<200则认为不是钢卷: 因为可能裁剪到车厢边缘的护栏
    auto is_valid_x_size = size_x > 200 && size_x < 2500;
    auto is_valid_y_size = size_y > 800 && size_y < 2500;
    auto is_valid_z_size = size_z > 100 && size_z < 1500;
   
    // 如果任意一个方向的大小不符合要求，则认为不是钢卷
    if (!is_valid_x_size || !is_valid_y_size || !is_valid_z_size) return false;
    
    // 下面的为旧的判断条件，新版本之前未使用
//    // 判断最高点和边界是否有一定距离
//    auto highest_point = cloud::getMaxZPoint(coil_cloud);
//    auto highest_to_min_distance = abs(highest_point.x - min_point.x);
//    auto highest_to_max_distance = abs(highest_point.x - max_point.x);
//    auto is_coil_min_length = highest_to_min_distance > config.is_coil_min_length_threshold &&
//                              highest_to_max_distance > config.is_coil_min_length_threshold;
//
//    // 判断钢卷边界任意点离车头或者车尾的距离是否小于300
//    auto is_coil_near_car_edge =
//            abs(min_point.x - (float) car_min_x) < 300 || abs(max_point.x - (float) car_max_x) < 300;
//
//    // 判断当两个条件都满足的情况下，则认为是离车厢很近而且长度很短的点云，不是钢卷
//    if (!is_coil_min_length && is_coil_near_car_edge) return false;

    // 默认认为是有效
    return true;
}

/**
 * 对单个钢卷点云进行拟合
 * @param coil_cloud 钢卷的点云
 * @param config 
 * @return 
 */
DetectResult Scan::fittingCoil(const pcl::PointCloud<pcl::PointXYZ>::Ptr &coil_cloud, const ScanConfig &config) {
    // 构造参数：用于调用圆柱体拟合算法
    std::vector<Vec3> points;
    for (auto &point: coil_cloud->points) {
        points.emplace_back(point.x, point.y, point.z);
    }

    // 拟合圆柱体: 用于识别钢卷
    auto fitting_algorithm = std::make_unique<Algorithm>();
    const int pNum = (int) points.size();
    auto *models = new double[7]{};
    fitting_algorithm->robustFitCylinder(points.data(), pNum, models);

    // 角度
    double axis_angle = atan2(models[3], models[4]) * 180 / M_PI;
    double diameter = isnan(models[6]) ? 0 : models[6] * 2;

    // 获取宽度: 目前拟合算法中不会直接获取宽度，所以需要手动计算 TODO: 但是当前的点云是经过裁剪的，所以这个宽度可能不准确
    auto size_y = std::get<1>(cloud::getCloudSize(coil_cloud));

    // 返回匹配结果
    return {0, COIL, models[0], models[1], models[2], size_y, diameter, axis_angle};
}

/**
 * 识别鞍座
 * @param saddles_cloud 
 * @param config 
 * @param coils_result
 * @return 
 */
std::vector<DetectResult>
Scan::detectSaddles(const pcl::PointCloud<pcl::PointXYZ>::Ptr &saddles_cloud,
                    const std::vector<DetectResult> &coils_result, const Carriage &carriage) {
    // 如果没有点云则返回空
    if (saddles_cloud->empty()) return {};

    // 执行sor降噪
    auto deNoise_saddles_cloud = cloud::sorFilter(saddles_cloud,
                                                  config.saddles_sor_mean_k,
                                                  config.saddles_sor_stddev_mul_thresh);
    saveCloudWhenDebug(deNoise_saddles_cloud, "deNoise_saddles_cloud.pcd");

    // 对点云进行聚类: 获取单个鞍座突出点的集合 
    auto feature_clusters = cloud::euclideanCluster(deNoise_saddles_cloud,
                                                    config.saddle_cluster_tolerance,
                                                    config.saddle_cluster_min_size);

    // 提取鞍座的特征点
    std::vector<pcl::PointXYZ> saddle_feature_points;
    for (auto &feature_cluster: feature_clusters) {
        // TODO: 目前从实际效果看，调用sor的时候如果点云的数量小于等于mean_k的值，程序会直接异常 
        if (config.saddle_cluster_min_size > config.saddle_sor_mean_k) {
            // TODO: 这里要对鞍座的点云进行基本的判的，避免是钢卷的干扰点导致
            // sor降噪
            feature_cluster = cloud::sorFilter(feature_cluster, config.saddle_sor_mean_k,
                                               config.saddle_sor_stddev_mul_thresh);
        }

        saddle_feature_points.push_back(getSaddleFeaturePoint(feature_cluster));
        saveCloudWhenDebug(feature_cluster, "feature_cluster.pcd");
    }

    // 保存特征点
    if (config.enable_debug_pcd) {
        // 将坐标链转换成点云
        auto feature_cloud = cloud::pointsToCloud(saddle_feature_points);
        saveCloudWhenDebug(feature_cloud, "saddle_feature_points.pcd");
    }

    // 使用模式匹配
    auto switched_patterns = Chassis::getFeatures(chassis);
    std::cout << "匹配模型数量: " << switched_patterns.size() << std::endl;
    auto match_results = getBestMatchPatterns(saddle_feature_points, switched_patterns);

    // 返回结果
    std::vector<DetectResult> results;
    for (auto &[chassis_index, points]: match_results) {
        auto match_chassis = chassis[chassis_index];
        auto centers = match_chassis.calcCenter(points);

        // z值采用车厢平面高度
        auto center_z = carriage.getPlaneCenter().z;

        for (auto &center: centers) {
            results.push_back({0, DetectType::SADDLE, center.x, center.y, center_z, 0, 0, 0});
        }
    }

    // 保存识别结果
    saveCloudWhenDebug(switchResultToCloud(results), "saddle_result_points.pcd");

    return results;
}

/**
 * 获取最佳匹配的模式，从所有的点中搜索出最佳多种组合
 * @param points 待查找的点集合 
 * @param patterns 参考模式集合 
 * @return 最佳的模式的索引和匹配的点集合 
 */
std::vector<std::pair<int, std::vector<std::pair<Point, Point>>>>
Scan::getBestMatchPatterns(const std::vector<pcl::PointXYZ> &points, std::vector<std::vector<Point>> &patterns) {
    // 转换入参
    auto switch_points = std::vector<Point>();
    for (auto &point: points) {
        switch_points.emplace_back(point.x, point.y, point.z);
    }

    // 执行匹配
    auto result = pattern::getBestMatchPatterns(switch_points, patterns);

    return result;

//    // 转换出参
//    std::vector<std::pair<int, std::vector<pcl::PointXYZ>>> results;
//    for (auto &item: result) {
//        std::vector<pcl::PointXYZ> temp_point;
//        for (auto &point: item.second) {
//            temp_point.emplace_back(point.x, point.y, point.z);
//        }
//        results.emplace_back(item.first, temp_point);
//    }
//    return results;
}

/**
 * 获取鞍座的特征点：从鞍座的突出的点云提取一个最具有代表性的点(往往是高点)
 * @param saddle_cloud 鞍座突出点的点云，一个鞍座可能有四个突出点，这里只需要其中一个 
 * @param config 配置信息 
 * @return 返回的是鞍座的特征点 
 */
pcl::PointXYZ
Scan::getSaddleFeaturePoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr &saddle_cloud) {
    // 获取最低点
    auto min_z_point = cloud::getMinZPoint(saddle_cloud);

    // 获取点云范围
    auto [min_point, max_point] = cloud::getMinMax(saddle_cloud);

    // 信任最高点的X值，Y取中间值，Z取最高点
    return { min_z_point.x, (min_point.y + max_point.y) / 2, min_z_point.z};
}

/**
 * 合并识别结果，库位和钢卷的识别需要相互印证，库位可能多识别出来，比如相同位置已经识别出了钢卷，那么这个时候下面就不应该再识别出库位
 * @param coils_cloud 
 * @param coils_result 
 * @param saddles_cloud 
 * @param saddles_result 
 * @param scan_config 
 * @return 
 */
std::vector<DetectResult>
Scan::mergeDetectResult(const pcl::PointCloud<pcl::PointXYZ>::Ptr &coils_cloud,
                        const std::vector<DetectResult> &coils_result,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr &saddles_cloud,
                        const std::vector<DetectResult> &saddles_result,
                        const MinMaxPoint &car_border,
                        const int car_index) {
    // 合并后的结果
    std::vector<DetectResult> results;

    // 根据钢卷结果剔除冗余的鞍座: 注意这步骤建议最先处理，避免后面的步骤过滤掉钢卷后导致鞍座保留，鞍座往往用于放卷，如果处理错误导致鞍座保留但是实际是有卷会导致风险
    auto filter_saddles = filterDetectSaddleWithCoil(coils_result, saddles_result,
                                                     config.min_saddle_coil_interval_x,
                                                     config.min_saddle_coil_interval_y);

    // 根据车厢范围剔除
    // 钢卷: 钢卷边缘目前保持和车厢最小200mm的距离
    auto filter_coils = filterDetectCoilWithCarBorder(coils_result, car_border,
                                                      config.min_coil_carriage_interval_x,
                                                      config.min_coil_carriage_interval_y);
    // 库位: 暂时设置成和车厢最小600mm的距离，至少保证基本能放卷
    filter_saddles = filterDetectSaddleWithCarBorder(filter_saddles, car_border,
                                                     config.min_saddle_carriage_interval_x,
                                                     config.min_saddle_carriage_interval_y);

    // 剔除明显异常的钢卷
    filter_coils = filterDetectCoilWithCoilParam(filter_coils, config.coil_max_width, config.coil_max_diameter);

    // 将车厢范围也添加到识别结果中
    auto car_header_result = DetectResult{car_index,
                                          DetectType::CAR_HEAD_POINT,
                                          car_border.first.x,
                                          car_border.first.y,
                                          car_border.first.z, 0, 0, 0};
    auto car_tail_result = DetectResult{car_index,
                                        DetectType::CAR_TAIL_POINT,
                                        car_border.second.x,
                                        car_border.second.y,
                                        car_border.second.z, 0, 0, 0};

    // 合并结果
    results.insert(results.end(), filter_coils.begin(), filter_coils.end());
    results.insert(results.end(), filter_saddles.begin(), filter_saddles.end());

    // 对结果按照X从小到大排序
    std::sort(results.begin(), results.end(), [](const DetectResult &a, const DetectResult &b) {
        return a.x < b.x;
    });

    results.push_back(car_header_result);
    results.push_back(car_tail_result);

    // 为所有的结果添加车厢索引
    for (auto &result: results) {
        result.car_index = car_index;
    }

    return results;
}


/**
 * @brief 格式化检测结果，正常不做任何处理，主要是根据特殊情况进行处理, 比如汽车的高低差的车厢
 * @param results 检测的结果
 * @return 格式化后的结果
 */
std::vector<DetectResult> Scan::formatDetectResult(const std::vector<DetectResult> &results) {
    return results;
}

/**
 * @brief 根据钢卷的结果剔除在钢卷范围内的库位，因为就算有钢卷也可能识别到鞍座，但是如果有卷则不应该返回鞍座，并且钢卷的识别相对准确，所以优先信任钢卷的识别结果
 * @param coil_results 钢卷识别结果
 * @param saddle_results 鞍座识别结果 
 * @param x_offset 判断范围额外要扩大的范围, 如果为负数则缩小范围 
 * @param y_offset 判断的范围额外要扩大的范围, 如果为负数则缩小范围 
 * @return 
 */
std::vector<DetectResult> Scan::filterDetectSaddleWithCoil(const std::vector<DetectResult> &coil_results,
                                                           const std::vector<DetectResult> &saddle_results,
                                                           float x_offset, float y_offset) {
    std::vector<DetectResult> filter_saddle_results;
    for (auto &saddle_result: saddle_results) {
        // 判断是否在钢卷范围内
        bool is_in_coil = false;
        for (auto &coil_result: coil_results) {
            // 计算钢卷的最大最小X、Y  TODO: 注意目前这里没有处理横着放的情况
            auto min_x = coil_result.x - coil_result.diameter / 2;
            auto max_x = coil_result.x + coil_result.diameter / 2;
            auto min_y = coil_result.y - coil_result.width / 2;
            auto max_y = coil_result.y + coil_result.width / 2;

            // 添加额外的范围：比如库位识别发现其在钢卷的200mm范围内，那么也是不合理的
            min_x = min_x - x_offset;
            max_x = max_x + x_offset;
            min_y = min_y - y_offset;
            max_y = max_y + y_offset;

            // 判断是否在钢卷范围内
            if (saddle_result.x > min_x && saddle_result.x < max_x &&
                saddle_result.y > min_y && saddle_result.y < max_y) {
                is_in_coil = true;
                break;
            }
        }

        // 如果不在钢卷范围内，则添加到结果中
        if (!is_in_coil) {
            filter_saddle_results.push_back(saddle_result);
        }
    }

    return filter_saddle_results;
}

/**
 * @brief 根据车厢范围以及偏移值，过滤钢卷识别结果
 * @param results 
 * @param car_border 
 * @param x_offset 
 * @param y_offset 
 * @return 
 */
std::vector<DetectResult> Scan::filterDetectCoilWithCarBorder(const std::vector<DetectResult> &results,
                                                              const MinMaxPoint &car_border,
                                                              float x_offset, float y_offset) {
    std::vector<DetectResult> filter_results;
    for (auto &result: results) {
        // 计算钢卷的最大最小X、Y  TODO: 注意目前这里没有处理横着放的情况
        auto min_x = result.x - result.diameter / 2;
        auto max_x = result.x + result.diameter / 2;
        auto min_y = result.y - result.width / 2;
        auto max_y = result.y + result.width / 2;

        // 判断是否在车厢范围内
        if (min_x > car_border.first.x + x_offset && max_x < car_border.second.x - x_offset &&
            min_y > car_border.first.y + y_offset && max_y < car_border.second.y - y_offset) {
            filter_results.push_back(result);
        }
    }

    return filter_results;
}

/**
 * @brief 根据车厢范围以及偏移值，过滤鞍座识别结果
 * @param results
 * @param car_border
 * @param x_offset
 * @param y_offset
 */
std::vector<DetectResult> Scan::filterDetectSaddleWithCarBorder(const std::vector<DetectResult> &results,
                                                                const MinMaxPoint &car_border,
                                                                float x_offset, float y_offset) {
    std::vector<DetectResult> filter_results;
    for (auto &result: results) {
        // 判断是否在车厢范围内
        if (result.x > car_border.first.x + x_offset && result.x < car_border.second.x - x_offset &&
            result.y > car_border.first.y + y_offset && result.y < car_border.second.y - y_offset) {
            filter_results.push_back(result);
        }
    }
    return filter_results;
}


/**
 * @brief 根据钢卷属性，过滤钢卷识别结果
 * @param results 钢卷识别结果 
 * @param max_coil_with 最大宽度
 * @param max_coil_diameter 最大直径
 * @return 
 */
std::vector<DetectResult> Scan::filterDetectCoilWithCoilParam(const std::vector<DetectResult> &results,
                                                              float max_coil_with,
                                                              float max_coil_diameter) {
    std::vector<DetectResult> filter_results;
    for (auto &result: results) {
        // 判断的时候稍微放大一点范围，因为拟合算法可能会有一些误差
        if (result.width < max_coil_with * 1.2 && result.diameter < max_coil_diameter * 1.2) {
            filter_results.push_back(result);
        }
    }

    return filter_results;
}

/**
 * 判断路径是文件还是目录，如果是文件则获取父目录，并且判断是否存在如果不存在则创建
 * @param path 文件路径
 */
void Scan::createDirectory(const std::string &path) {
    auto path_info = boost::filesystem::path(path);

    // 判断路径是文件还是目录，如果是文件则获取父目录，并且判断是否存在如果不存在则创建 
    auto parent_path = path_info.extension().empty() ? path_info : path_info.parent_path();

    // 创建目录
    if (!boost::filesystem::exists(parent_path)) {
        boost::filesystem::create_directories(parent_path);
    }
}

/**
 * 保存点云到pcd文件
 * @param cloud 
 * @param path 
 * @param is_binary 
 */
void Scan::saveCloudToPcd(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &path, bool is_binary) {
    // 如果点云为空则不保存
    if (cloud->empty()) {
        return;
    }

    // 获取路径的绝对路径
    auto absolute_path = boost::filesystem::absolute(path).string();

    // 先尝试创建目录
    createDirectory(absolute_path);

    // 保存点云到pcd文件
    pcl::io::savePCDFile(absolute_path, *cloud, is_binary);
}


/**
 * 当属于调试模式的时候，保存点云到文件
 * @param cloud 
 * @param name 
 */
void Scan::saveCloudWhenDebug(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &name) const {
    // 如果开启了日志，则保存点云到文件
    if (config.enable_debug_pcd) {
        saveCloudToPcd(cloud, name, false);
    }
}

/**
 * 将检测结果转换为点
 * @param result 检测结果
 * @return 点的集合 
 */
std::vector<Point> Scan::detectResultToPoint(const std::vector<DetectResult> &result) {
    std::vector<Point> points;
    for (auto &item: result) {
        points.emplace_back(static_cast<float>(item.x), static_cast<float>(item.y), static_cast<float>(item.z));
    }
    return points;
}

/**
 * 创建点云
 * @param points 根据点的集合创建点云
 * @return 点云对象 
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr Scan::createCloud(const std::vector<Point> &points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto &point: points) {
        cloud->push_back({point.x, point.y, point.z});
    }
    return cloud;
}

/**
 * 保存点到pcd文件
 * @param points 
 * @param path 
 */
void Scan::savePointToPcd(const std::vector<Point> &points, const std::string &path) {
    // 创建点云
    auto cloud = createCloud(points);

    // 保存点云到文件
    saveCloudToPcd(cloud, path);
}


/**
 * @brief 自动缩放点云，主要是输入的点云先前是存在单位不同的情况，部分是m，而最新的是mm，所以尝试根据期望的大小判断是否是m，如果是m则转换为mm
 * @param cloud_in 输入点云 
 * @param excepted_x_size 期望的X方向长度
 * @param excepted_y_size 期望的Y方向长度
 * @param excepted_z_size 期望的Z方向长度
 * @return 缩放后的点云
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr
Scan::autoScaleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                     float excepted_x_size,
                     float excepted_y_size,
                     float excepted_z_size) {
    // 获取大小
    auto [size_x, size_y, size_z] = cloud::getCloudSize(cloud_in);
    if (size_x < size_y)
    {
        auto temp = size_y;
        size_y = size_x;
        size_x = temp;
    }

    // 如果实际大小小于期望大小的500倍，则放大1000倍
    // 不是判断小于1000倍是因为实际的点云可能比期望的大写，如果直接用1000倍可能判断错误，但是因为m和mm本来就差了1000倍的数量级，500判断能满足要求
    auto scale_x = size_x * 500 < excepted_x_size;
    auto scale_y = size_y * 500 < excepted_y_size;
    // auto scale_z = size_z * 500 < excepted_z_size; // Z不做判断，因为点云在扫描头附近存在干扰，所以Z方向的大小可能会比较大

    std::cout << "点云X范围: " << size_x << "   期望X范围: " << excepted_x_size << "   缩放X: " << scale_x << "      Y范围: " << size_y << "   期望Y范围 : " << excepted_y_size << "   缩放Y : " << scale_y << std::endl;

    // 如果需要将X、Y、Z坐标*1000转换成mm
    if (scale_x && scale_y) {
        return cloud::scaleCloud(cloud_in, 1000);
    }

    //return cloud::offsetCloud(cloud_in, 0,2700,9330);
    return cloud_in;
}


/**
 * @brief 格式化检测结果，主要是统一格式化序号等，以及基本的识别结果检查
 * @param results 检测的结果 
 * @return 转换后的结果 
 */
void Scan::checkDetectResults(const std::vector<DetectResult> &results) const {
    // 遍历判断相邻的车头的X差距是否满足最小的车厢间隔
    auto car_results = getSortedDetectResultsByTypes(results, {CAR_HEAD_POINT});
    for (auto i = 0; i < static_cast<int>(car_results.size()) - 1; i++) {
        auto &car_result = car_results[i];
        auto &next_car_result = car_results[i + 1];
        if (next_car_result.x - car_result.x < config.min_carriage_interval) {
            auto error_message = "车厢" + std::to_string(car_result.car_index) + "和车厢" +
                                 std::to_string(next_car_result.car_index) + "的间隔小于最小车厢间隔" +
                                 std::to_string(config.min_carriage_interval);
            std::cout << error_message << std::endl;
//            throw std::runtime_error(error_message);
        }
    }

    // 遍历获取所有库位和钢卷的检测结果，判断是否满足最小的库位间隔要求
    auto coil_results = getSortedDetectResultsByTypes(results, {COIL, SADDLE});
    for (auto i = 0; i < static_cast<int>(coil_results.size()) - 1; i++) {
        auto &coil_result = coil_results[i];
        auto &next_coil_result = coil_results[i + 1];
        if (next_coil_result.x - coil_result.x < config.min_saddle_interval) {
            auto error_message = "库位" + std::to_string(coil_result.x) + "和库位" +
                                 std::to_string(next_coil_result.x) + "的间隔小于最小库位间隔" +
                                 std::to_string(config.min_saddle_interval);
            std::cout << error_message << std::endl;
//            throw std::runtime_error(error_message);
        }
    }

    // 所有库位是否包含在某个车厢内
}

/**
 * @brief 根据类型获取检测结果
 * @param results 检测的结果 
 * @param types 类型 
 * @return 
 */
[[nodiscard]] std::vector<DetectResult>
Scan::getSortedDetectResultsByTypes(const std::vector<DetectResult> &results, const std::vector<DetectType> &types) {
    std::vector<DetectResult> detect_results;
    for (auto &result: results) {
        if (std::find(types.begin(), types.end(), result.data_type) != types.end()) {
            detect_results.push_back(result);
        }
    }

    // 按照X排序
    std::sort(detect_results.begin(), detect_results.end(), [](const DetectResult &a, const DetectResult &b) {
        return a.x < b.x;
    });

    return detect_results;
}

/**
 * @brief 根据配置信息中的配置尝试交换点云的xy坐标，因为目前算法中全部是假定X轴是车厢的长度方向，Y轴是宽度方向，但是有些点云可能是反的，所以需要尝试交换
 * @param cloud_in 输入点云 
 * @return 交换后的点云 
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr
Scan::trySwitchCloudXy(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, int& switch_flag) const {
    auto axis = config.carAxis(cloud_in);
    std::string logStr;
    if (axis == X)
        logStr = "点云方向为X轴，不需要交换";
    else if (axis == Y)
        logStr = "点云方向为Y轴，需要交换";
    else
        logStr = "点云方向为Z轴，不支持";
    std::cout << "点云方向为: " << logStr << std::endl;

    // 根据车厢的轴向进行交换
    switch (axis) {
        // X轴默认就是车厢的长度方向，所以不需要交换
        case X: {
            switch_flag = Axis::X;
            return cloud_in;
        }
            // Y轴是车厢的宽度方向，所以需要交换
        case Y: {
            switch_flag = Axis::Y;
            return switchXy(cloud_in);
        }
        case Z: {
            switch_flag = Axis::Z;
            throw std::runtime_error("车厢的轴向不能是Z轴");
        }
        default: {
            switch_flag = -1;
            throw std::runtime_error("未知的车厢轴向");
        }
    }
}


/**
 * @brief 尝试交换检测结果的xy坐标, 和trySwitchCloudXy中的逻辑需要保持一致
 * @param results 检测的结果 
 * @return 交换后的检测结果 
 */
std::vector<DetectResult>
Scan::trySwitchResultXy(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                        const std::vector<DetectResult> &results, int switch_flag) const {

    switch (switch_flag) {
        // X轴默认就是车厢的长度方向，所以不需要交换
        case X: {
            return results;
        }
            // Y轴是车厢的宽度方向，所以需要交换
        case Y: {
            cloud_in = switchXy(cloud_in);
            return switchXy(results);
        }
        case Z: {
            throw std::runtime_error("车厢的轴向不能是Z轴");
        }
        default: {
            throw std::runtime_error("未知的车厢轴向");
        }
    }
}


/**
 * @brief 交换点云的xy坐标
 * @param cloud_in 输入点云 
 * @return 交换后的点云 
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr
Scan::switchXy(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_switched(new pcl::PointCloud<pcl::PointXYZ>);
    // 拷贝点云
    pcl::copyPointCloud(*cloud_in, *cloud_switched);

    // 交换坐标
    for (auto &point: cloud_switched->points) {
        std::swap(point.x, point.y);
    }

    return cloud_switched;
}


/**
 * @brief 交换检测结果的xy坐标
 * @param results 检测的结果 
 * @return 交换后的检测结果 
 */
std::vector<DetectResult>
Scan::switchXy(const std::vector<DetectResult> &results) {
    std::vector<DetectResult> results_switched;
    for (auto &result: results) {
        float axis = formatAngle(90 - result.axis, 0);
        results_switched.push_back(
                {result.car_index, result.data_type, result.y, result.x, result.z, result.width, result.diameter,
                 axis });
    }
    return results_switched;
}

/**
 * @brief 将点转换为点云
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr Scan::switchResultToCloud(std::vector<pcl::PointXYZ> &in_points) {
    std::vector<Point> points;
    for (auto &result: in_points) {
        points.emplace_back(result.x, result.y, result.z);
    }
    return createCloud(points);
}

/**
 * @brief 将检测结果转换为点云
 * @param results 
 * @return 
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr Scan::switchResultToCloud(std::vector<DetectResult> &results) {
    std::vector<Point> points;
    for (auto &result: results) {
        points.emplace_back(result.x, result.y, result.z);
    }
    return createCloud(points);
}

/**
 * @brief 查找从X轴看，当Z最小(取一段X界面，最高的Z)的点对应的X
 * @param cloud_in 
 * @return 
 */
pcl::PointXYZ Scan::findFirstMinZPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in) {
    // X从小遍历，每次取50范围内的点云，找到最高的Z，并将这个Z和X记录下来
    auto [min_point, max_point] = cloud::getMinMax(cloud_in);
    auto min_x = static_cast<int>(min_point.x) + 200;   // 缩减一点，避免边上有比较低的位置，导致找到外面的低点，而不是中心位置的低点
    auto max_x = static_cast<int>(max_point.x) - 200;   // 缩减一点，避免边上有比较低的位置，导致找到外面的低点，而不是中心位置的低点

    // 从X最小的开始遍历: 并将每段的X以及对大的Z记录下来
    auto x_and_z = std::vector<std::pair<int, pcl::PointXYZ>>();

    for (auto x = min_x; x < max_x; x += 50) {
        auto cloud_x = cloud::cropCloudX(cloud_in, static_cast<float>(x), static_cast<float>(x + 50));
        if (cloud_x->empty()) continue;
        auto max_z_point = cloud::getMaxZPoint(cloud_x);
        x_and_z.emplace_back(x, max_z_point);
    }

    // 计算Z的平均值
    auto z_sum = std::accumulate(x_and_z.begin(), x_and_z.end(), 0.0f,
                                 [](float sum, const std::pair<int, pcl::PointXYZ> &point) {
                                     return sum + point.second.z;
                                 });
    auto z_avg = z_sum / static_cast<float>(x_and_z.size());

    // 计算Z最小值
    auto min_z = std::numeric_limits<float>::max();
    for (auto &point: x_and_z) {
        if (point.second.z < min_z) {
            min_z = point.second.z;
        }
    }

    // 获取首个和最小Z差值在20范围的点，直接返回
    for (auto &point: x_and_z) {
        if (abs(point.second.z - min_z) < 20) {
            return point.second;
        }
    }

//    // 计算斜率，取斜率最小的点的X作为返回值
//    auto min_slope = std::numeric_limits<float>::max();
//    auto min_slope_point = x_and_z[0].second;
//
//    // 由于是需要找到第一个最小的Z，所以需要设置个阈值，避免找到全局最小的Z
//    auto slope_threshold = 0.1;
//
//    for (int i = 0; i < static_cast<int>(x_and_z.size()) - 1; i++) {
//        auto &point1 = x_and_z[i];
//        auto &point2 = x_and_z[i + 1];
//        auto slope = abs((point2.second.z - point1.second.z) / (point2.second.x - point1.second.x));
//        if (slope < min_slope) {
//            min_slope = slope;
//            min_slope_point = point1.second;
//
//            // 如果斜率小于阈值，而且Z小于平均值，那么认为是首个最小的Z
//            if (min_slope < slope_threshold && min_slope_point.z < z_avg) {
//                return min_slope_point;
//            }
//        }
//    }
//
//    return min_slope_point;
}


/**
 * @brief 格式化输出点云，默认实现，直接返回
 * @param cloud: 输入点云
 * @param cloud_out: 输出用的点云
 * @param results: 识别的结果，用于辅助裁剪
 */
void
Scan::formatOutputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ> *cloud_out,
                        const std::vector<DetectResult> &results) {
    copyPointCloud(*cloud, *cloud_out);
}

// 单位化向量
Eigen::Vector3f Scan::normalize(const Eigen::Vector3f& v) {
    // 计算向量长度
    double len = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (len > 0) {
        return v / len;
    }
    return v;
}

// 计算内缩后的边界点
std::vector<Eigen::Vector3f> Scan::shrinkQuadrilateral(const std::vector<Eigen::Vector3f>& original_points, float shrink_distance) {
    std::vector<Eigen::Vector3f> shrunk_points;

    if (original_points.size() != 4) {
        std::cerr << "需要4个边界点!" << std::endl;
        return shrunk_points;
    }

    // 计算中心点
    Eigen::Vector3f center(0, 0, 0);
    for (const auto& p : original_points) {
        center += p;
    }
    center /= 4.0f;

    // 对每个点进行内缩
    for (const auto& point : original_points) {
        // 计算从边界点指向中心点的方向向量
        Eigen::Vector3f direction = center - point;

        // 单位化方向向量
        Eigen::Vector3f unit_direction = normalize(direction);

        // 计算内缩后的点
        Eigen::Vector3f shrunk_point = point + unit_direction * shrink_distance;
        shrunk_points.push_back(shrunk_point);
    }

    return shrunk_points;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Scan::cropPointCloudWithQuadrilateral(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, const std::vector<Eigen::Vector3f>& boundary_points)
{
    // 创建凸包点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 将边界点转换为PCL点云
    for (const auto& point : boundary_points) {
        pcl::PointXYZ p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        hull_cloud->points.push_back(p);
    }

    // 创建凸多边形点云（需要闭合，所以添加第一个点）
    pcl::PointXYZ first_point = hull_cloud->points[0];
    hull_cloud->points.push_back(first_point);

    // 使用CropHull滤波器
    pcl::CropHull<pcl::PointXYZ> crop_hull;
    crop_hull.setInputCloud(input_cloud);
    crop_hull.setHullCloud(hull_cloud);
    crop_hull.setDim(2); // 2D凸包

    // 设置裁剪参数
    std::vector<pcl::Vertices> polygons;
    pcl::Vertices polygon;
    for (size_t i = 0; i < hull_cloud->size(); ++i) {
        polygon.vertices.push_back(i);
    }
    polygons.push_back(polygon);
    crop_hull.setHullIndices(polygons);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    crop_hull.filter(*cropped_cloud);

    return cropped_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Scan::getPointCloudDifference(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_a, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_b, float tolerance)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr difference_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 创建KdTree用于快速搜索
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_b);

    // 搜索参数
    std::vector<int> point_idx(1);
    std::vector<float> point_squared_distance(1);

    // 遍历cloud_a中的每个点
    for (const auto& point : cloud_a->points) {
        // 在cloud_b中搜索最近点
        if (kdtree.nearestKSearch(point, 1, point_idx, point_squared_distance) > 0) {
            // 如果最近距离大于容差，说明该点不在cloud_b中
            if (point_squared_distance[0] > tolerance * tolerance) {
                difference_cloud->points.push_back(point);
            }
        }
        else {
            // 如果没有找到最近点，也加入到差集
            difference_cloud->points.push_back(point);
        }
    }

    difference_cloud->width = difference_cloud->points.size();
    difference_cloud->height = 1;
    difference_cloud->is_dense = true;

    return difference_cloud;

}

std::vector<DetectResult> Scan::detectHighsidedSaddles(pcl::PointCloud<pcl::PointXYZ>::Ptr& saddles_cloud, const std::vector<DetectResult>& coils)
{
    // 如果没有点云则返回空
    if (saddles_cloud->empty()) return {};

    std::vector<DetectResult> highsided_saddles;

    // 执行sor降噪
    auto deNoise_saddles_cloud = cloud::sorFilter(saddles_cloud,
        config.saddles_sor_mean_k,
        config.saddles_sor_stddev_mul_thresh);
    // 去除离群点
    auto removed_outliers_cloud = cloud::sorFilter(deNoise_saddles_cloud, 100, 2.0);
    saveCloudWhenDebug(removed_outliers_cloud, "12-降噪并去除离群点后所有鞍座点云.pcd");

    // 获取单个鞍座点云
    auto saddle_clusters = cloud::euclideanCluster(removed_outliers_cloud,
        config.highsided_saddle_cluster_tolerance,
        config.saddle_cluster_min_size);

    // 分类鞍座及可能的部分鞍座
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> saddle_clouds;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> part_saddle_clouds;
    int idx = 0;
    for (auto& cluster : saddle_clusters)
    {
        // 再次去除离群点
        cluster = cloud::sorFilter(cluster, 50, 3.0);
        // 获取点云的矩形相似度
        int like_rect = calculateRotatedRectCoverage(cluster);
        std::string path = "13-" + std::to_string(idx) + "号待判断鞍座.pcd";
        saveCloudWhenDebug(cluster, path);
        // 获取鞍座信息
        auto [min_point, max_point] = cloud::getMinMax(cluster);
        double saddle_width = max_point.y - min_point.y;
        double saddle_length = max_point.x - min_point.x;
        double saddle_height = max_point.z - min_point.z;
        double length_width_ratio = saddle_length / saddle_width;
        std::cout << "鞍座" << std::to_string(idx) << " : 长度 : " << std::to_string(saddle_length) << "  宽度: " << std::to_string(saddle_width) << "   长宽比: " << std::to_string(length_width_ratio) << "   高度: " << std::to_string(saddle_height) << "矩形相似度: " << std::to_string(like_rect) << std::endl;
        // 如果长度合格、长宽比合格、高度合格、矩形相似度合格，则认为是鞍座
        bool length_ok = saddle_length > config.saddle_length_min;
        bool length_width_ratio_ok = length_width_ratio > config.saddle_length_width_ratio_min && length_width_ratio < config.saddle_length_width_ratio_max;
        bool height_ok = saddle_height > config.saddle_height_min && saddle_height < config.saddle_height_max;
        bool like_rect_ok = like_rect > config.saddle_like_rect_min;
        std::cout << "鞍座聚类: " << idx << ": 长度: " << saddle_length << " 长度合格信息: " << length_ok << "  长宽比: " << length_width_ratio << " 长宽比合格信息: " << length_width_ratio_ok << "  高度: " << saddle_height << " 高度合格信息: " << height_ok << "  矩形相似度: " << like_rect << " 矩形相似度合格信息: " << like_rect_ok << std::endl;
        if (saddle_length > config.saddle_length_min && (length_width_ratio > config.saddle_length_width_ratio_min && length_width_ratio < config.saddle_length_width_ratio_max) && (saddle_height > config.saddle_height_min && saddle_height < config.saddle_height_max) && like_rect > config.saddle_like_rect_min)
        {
            saddle_clouds.push_back(cluster);
        }
        else
        {
            if (like_rect > config.part_saddle_like_rect_min)
            {
                part_saddle_clouds.push_back(cluster);
            }
        }
        idx++;
    }

    // 合并断裂鞍座
    for (auto& part_saddle : part_saddle_clouds)
    {
        for (auto& saddle_cloud : saddle_clouds)
        {
            // 获取部分鞍座信息
            auto [part_min_point, part_max_point] = cloud::getMinMax(part_saddle);
            float part_saddle_width = part_max_point.y - part_min_point.y;
            // 获取鞍座信息
            auto [min_point, max_point] = cloud::getMinMax(saddle_cloud);
            float saddle_width = max_point.y - min_point.y;
            std::cout << "鞍座与部分鞍座信息匹配: 宽度: [" << saddle_width << ", " << part_saddle_width << "] Y两侧距离: [" << abs(part_max_point.y - max_point.y) << ", " << abs(part_min_point.y - min_point.y) << "] X距离: [" << abs(part_min_point.x - max_point.x) << ", " << abs(part_max_point.x - min_point.x) << std::endl;
            // 宽度相似、y有一侧位置相似、x距离近
            if ((abs(part_saddle_width - saddle_width) < config.part_saddle_width_min_gap) && (abs(part_min_point.y - min_point.y) < config.part_saddle_y_min_gap || abs(part_max_point.y - max_point.y) < config.part_saddle_y_min_gap) && (abs(part_min_point.x - max_point.x) < config.part_saddle_max_gap || abs(part_max_point.x - min_point.x) < config.part_saddle_max_gap))
            {
                *saddle_cloud = *saddle_cloud + *part_saddle;
                break;
            }
        }
    }

    // 识别所有鞍座
    idx = 0;
    for (auto& cluster : saddle_clouds)
    {
        std::string path = "14-" + std::to_string(idx) + "号合并后鞍座.pcd";
        saveCloudWhenDebug(cluster, path);
        auto [min_point, max_point] = cloud::getMinMax(cluster);
        double center_x = (min_point.x + max_point.x) / 2.0;
        double center_y = (min_point.y + max_point.y) / 2.0;
        double center_z = 0;
        // 判断是否与钢卷重叠
        bool is_in_coil = false;
        for (auto& coil_result : coils) {
            // 计算钢卷的最大最小X、Y  TODO: 注意目前这里没有处理横着放的情况
            auto min_x = coil_result.x - coil_result.width / 2;
            auto max_x = coil_result.x + coil_result.width / 2;
            auto min_y = coil_result.y - coil_result.diameter / 1.5;
            auto max_y = coil_result.y + coil_result.diameter / 1.5;

            // 添加额外的范围：比如库位识别发现其在钢卷的200mm范围内，那么也是不合理的
            min_x = min_x - config.min_saddle_coil_interval_x;
            max_x = max_x + config.min_saddle_coil_interval_x;
            min_y = min_y - config.min_saddle_coil_interval_y;
            max_y = max_y + config.min_saddle_coil_interval_y;

            std::cout << "鞍座中心: (" << center_x << ", " << center_y << ")  钢卷中心: (" << coil_result.x << ", " << coil_result.y << ")  钢卷范围: (" << min_x << ", " << max_x << ", " << min_y << ", " << max_y << ")" << std::endl;

            // 判断是否在钢卷范围内
            if (center_x > min_x && center_x < max_x &&
                center_y > min_y && center_y < max_y) {
                is_in_coil = true;
                std::cout << "鞍座 " << std::to_string(idx) << " 与钢卷 " << std::to_string(coil_result.car_index) << " 重叠" << std::endl;
                break;
            }
        }

        // 如果不在钢卷范围内，则计算高度
        if (!is_in_coil) {
            auto cropped_cluster = cloud::cropCloud(cluster,
                center_x - 50, center_x + 50,
                center_y - 50, center_y + 50,
                min_point.z, max_point.z);
            center_z = getGlobalAverageHeight(cropped_cluster);
            double angle = computeMinBoundingBoxAnglePCA(cluster);
            highsided_saddles.push_back({ 0, DetectType::SADDLE, center_x, center_y, center_z, 0, 0, formatAngle(angle, 90)});
        }
        idx++;
    }

    return highsided_saddles;
}

float Scan::getGlobalAverageHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    if (cloud->empty()) {
        std::cout << "点云为空！" << std::endl;
        return 0.0f;
    }

    float sum_z = 0.0f;
    for (const auto& point : *cloud) {
        sum_z += point.z;
    }

    float avg_height = sum_z / cloud->size();
    std::cout << "全局平均高度: " << avg_height << " (基于 " << cloud->size() << " 个点)" << std::endl;
    return avg_height;
}

float Scan::computeMinBoundingBoxAnglePCA(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
    //pcl::PointCloud<pcl::PointXYZ>::Ptr compressed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //*compressed_cloud = *cloud_in;

    //for (auto& point : *compressed_cloud) {
    //    point.z = 0;  // 将所有点压缩到Z=0平面
    //}

    //pcl::PCA<pcl::PointXYZ> pca;
    //pca.setInputCloud(compressed_cloud);

    //Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
    //Eigen::Vector3f main_direction = eigenvectors.col(0);  // 第一主成分

    //// 计算与X轴的夹角
    //float angle = atan2(main_direction.y(), main_direction.x());

    ////// 规范化角度到 [0, π) 范围
    ////if (angle < 0) angle += M_PI;
    ////if (angle >= M_PI) angle -= M_PI;

    //return angle * 180 / M_PI;

    pcl::PointCloud<pcl::PointXYZ>::Ptr compressed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *compressed_cloud = *cloud_in;

    for (auto& point : *compressed_cloud) {
        point.z = 0;
    }

    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(compressed_cloud);
    Eigen::Matrix3f eigenvectors = pca.getEigenVectors();

    // 选择与X轴夹角最小的主轴
    Eigen::Vector3f x_axis(1, 0, 0);
    float min_angle = M_PI;
    Eigen::Vector3f selected_direction;

    for (int i = 0; i < 2; ++i) {  // 只检查前两个主成分（第三个是Z方向）
        Eigen::Vector3f dir = eigenvectors.col(i);
        float angle = acos(fabs(dir.dot(x_axis)));  // 计算与X轴的夹角

        if (angle < min_angle) {
            min_angle = angle;
            selected_direction = dir;
        }
    }

    // 计算选定方向与X轴的夹角
    float angle = atan2(selected_direction.y(), selected_direction.x());
    return angle * 180 / M_PI;
}

int Scan::calculateRotatedRectCoverage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    if (cloud->empty()) return 0;

    try {
        // 1. PCA分析找到主方向
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(cloud);
        Eigen::Matrix3f eigenvectors = pca.getEigenVectors();

        // 2. 将点云旋转到主方向（使点云对齐坐标轴）
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pca.project(*cloud, *aligned_cloud);

        // 3. 计算旋转后的AABB（这就是最小包围矩形）
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*aligned_cloud, min_pt, max_pt);

        double rect_width = max_pt.x - min_pt.x;
        double rect_height = max_pt.y - min_pt.y;
        double rect_area = rect_width * rect_height;

        if (rect_area <= 0) return 0;

        // 4. 将矩形划分为网格，统计有点的网格数量
        int grid_size = 10; // 默认网格大小

        if (cloud->size() < 30) {
            grid_size = 5;   // 点数很少，用小网格
        }
        else if (cloud->size() < 100) {
            grid_size = 8;   // 点数较少，用中等网格
        }
        std::vector<std::vector<bool>> grid(grid_size, std::vector<bool>(grid_size, false));

        double cell_width = rect_width / grid_size;
        double cell_height = rect_height / grid_size;

        int filled_cells = 0;

        for (const auto& point : aligned_cloud->points) {
            int x_index = static_cast<int>((point.x - min_pt.x) / cell_width);
            int y_index = static_cast<int>((point.y - min_pt.y) / cell_height);

            x_index = std::max(0, std::min(grid_size - 1, x_index));
            y_index = std::max(0, std::min(grid_size - 1, y_index));

            if (!grid[x_index][y_index]) {
                grid[x_index][y_index] = true;
                filled_cells++;
            }
        }

        // 5. 计算覆盖率
        double coverage = static_cast<double>(filled_cells) / (grid_size * grid_size);
        return int(coverage * 100.0);

    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 0;
    }
}

float Scan::formatAngle(float angle, int offset)
{
    angle += offset;
    // 先规范到 [0, 180) 度
    angle = fmod(angle, 180.0f);
    if (angle < -10.0f) angle += 180.0f;

    return angle;
}

float Scan::computeUnwindingAngleFromSaddle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    if (!cloud || cloud->empty()) return 0.0f;

    // 1. 按X坐标分割左右两侧鞍座
    pcl::PointCloud<pcl::PointXYZ>::Ptr left_saddle(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr right_saddle(new pcl::PointCloud<pcl::PointXYZ>);

    // 找到X坐标的中值来分割左右
    std::vector<float> x_values;
    for (const auto& point : *cloud) {
        x_values.push_back(point.x);
    }
    std::sort(x_values.begin(), x_values.end());
    float x_median = x_values[x_values.size() / 2];

    // 分割左右鞍座
    for (const auto& point : *cloud) {
        if (point.x < x_median - 0.1f) {  // 留一点缓冲
            left_saddle->push_back(point);
        }
        else if (point.x > x_median + 0.1f) {
            right_saddle->push_back(point);
        }
    }

    // 2. 提取每侧的最高点（按Y坐标）
    std::vector<pcl::PointXYZ> highest_points;

    // 左侧最高点
    if (!left_saddle->empty()) {
        auto max_left = std::max_element(left_saddle->begin(), left_saddle->end(),
            [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
                return a.y < b.y;
            });
        highest_points.push_back(*max_left);
    }

    // 右侧最高点  
    if (!right_saddle->empty()) {
        auto max_right = std::max_element(right_saddle->begin(), right_saddle->end(),
            [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
                return a.y < b.y;
            });
        highest_points.push_back(*max_right);
    }

    // 3. 如果找到至少2个最高点，拟合直线
    if (highest_points.size() >= 2) {
        // 计算直线斜率
        float dx = highest_points[1].x - highest_points[0].x;
        float dy = highest_points[1].y - highest_points[0].y;

        if (fabs(dx) > 1e-6) {  // 避免除零
            float angle = atan2(dy, dx) * 180.0f / M_PI;

            // 规范到 [-90, 90] 范围
            if (angle > 90.0f) angle -= 180.0f;
            if (angle < -90.0f) angle += 180.0f;

            return angle;
        }
    }

    return 0.0f;  // 默认值

    //// 2. 提取每侧Y坐标最高的前N个点
    //const int top_n = 10;  // 每侧取10个最高点

    //pcl::PointCloud<pcl::PointXYZ>::Ptr top_points(new pcl::PointCloud<pcl::PointXYZ>);

    //// 左侧最高区域
    //if (!left_saddle->empty()) {
    //    std::sort(left_saddle->begin(), left_saddle->end(),
    //        [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
    //            return a.y > b.y;  // 降序排列
    //        });

    //    for (int i = 0; i < std::min(top_n, (int)left_saddle->size()); ++i) {
    //        top_points->push_back(left_saddle->at(i));
    //    }
    //}

    //// 右侧最高区域
    //if (!right_saddle->empty()) {
    //    std::sort(right_saddle->begin(), right_saddle->end(),
    //        [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
    //            return a.y > b.y;
    //        });

    //    for (int i = 0; i < std::min(top_n, (int)right_saddle->size()); ++i) {
    //        top_points->push_back(right_saddle->at(i));
    //    }
    //}

    //// 3. 使用RANSAC拟合直线
    //if (top_points->size() >= 2) {
    //    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    //    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    //    pcl::SACSegmentation<pcl::PointXYZ> seg;
    //    seg.setOptimizeCoefficients(true);
    //    seg.setModelType(pcl::SACMODEL_LINE);
    //    seg.setMethodType(pcl::SAC_RANSAC);
    //    seg.setDistanceThreshold(0.01);  // 调整阈值
    //    seg.setInputCloud(top_points);
    //    seg.segment(*inliers, *coefficients);

    //    if (inliers->indices.size() >= 2) {
    //        // 从直线系数提取角度
    //        // 直线方程: point_on_line.x + lambda * direction.x
    //        float dir_x = coefficients->values[3];
    //        float dir_y = coefficients->values[4];

    //        float angle = atan2(dir_y, dir_x) * 180.0f / M_PI;

    //        // 规范到 [-90, 90]
    //        if (angle > 90.0f) angle -= 180.0f;
    //        if (angle < -90.0f) angle += 180.0f;

    //        return angle;
    //    }
    //}

    //return 0.0f;
}