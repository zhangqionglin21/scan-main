#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include "car_scan.h"
#include "extensions.h"

/**
 * @brief 构造函数
 */
CarScan::CarScan() : Scan(getDefaultConfig()) {
}


/**
 * @brief 指定默认配置的构造函数
 */
CarScan::CarScan(const ScanConfig &config) : Scan(config) {

}


/**
 * @brief 从整体的点云中提取出车厢信息，包含车厢部分的点云（一般不包含围栏），车厢的边界信息
 * @param cloud_in 输入的点云
 * @return 
 */
std::vector<Carriage> CarScan::getCarriages(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in) {
    std::cout << "使用CarScan::getCarriages检测" << std::endl;
    auto carriages = std::vector<Carriage>();
    auto cluster_clouds = cloud::euclideanCluster(cloud_in, 100, 10);
    std::cout << "聚类数量: " << cluster_clouds.size() << std::endl;

    for (auto &cluster_cloud: cluster_clouds) {
        try {
            // 获取点云的大小
            auto [min_point, max_point] = cloud::getMinMax(cluster_cloud);
            auto [x_size, y_size, z_size] = cloud::getXyzSize(min_point, max_point);
            std::cout << "聚类点云大小: " << x_size << " " << y_size << " " << z_size << std::endl;
            std::cout << "车厢筛选大小: " << config.car_length << " " << config.car_width << " " << config.car_height << std::endl;

            // 判断是否在车身长度、宽度的范围内
            if (x_size > (config.car_length * 0.8) && y_size > (config.car_width * 0.8) &&
                max_point.z > (config.car_height * 0.8)) {

                // 获取点云大小，将X方向裁剪300，避免车头车尾围栏干扰
                auto cropped_cluster_cloud = cloud::cropCloudX(cluster_cloud, min_point.x + 300, max_point.x - 300);
                saveCloudWhenDebug(cropped_cluster_cloud, "04-裁剪车厢头尾后的点云.pcd");

                // 判是否是高低差的车厢
                auto [mutation_x, mutation_z_range] = cloud::findMutationMinZX(cropped_cluster_cloud);
                std::cout << "车厢高低差: " << mutation_z_range << std::endl;
                // 如果存在突变的地方，而且高低差超过200, 则认为是高低差距的车厢
                if (mutation_z_range > 200 &&
                    // 正常高低差的车厢突变位置一般再前1/3左右的位置，但是车厢头部发护板有可能也会识别为突变，所以这里加上一个限制，避免将普通的车厢识别为高低差的车厢
                    std::abs(mutation_x - min_point.x) > 1500 &&
                    std::abs(mutation_x - max_point.x) > 1500) { // 从目前的车厢看实际高度差在500左右
                    // 根据突变的X值，将点云分为两部分（增加100）
                    auto left_cloud = cloud::cropCloudX(cluster_cloud, min_point.x, mutation_x + 100);
                    auto right_cloud = cloud::cropCloudX(cluster_cloud, mutation_x - 100, max_point.x);

                    // 获取车厢信息
                    auto left_carriage = getCarriage(cloud_in, left_cloud);
                    auto right_carriage = getCarriage(cloud_in, right_cloud);

                    // 修改下范围：将左侧天车的最大的X额外加200，将右侧天车的最小X额外加200
                    // 增加的原因是高低的车厢本身是一个整体，但是由于突变的地方，导致分开了，而车厢的判断流程中会判断卷是否离车厢太近
                    // 如果不增加可能导致离分割位置比较近的钢卷被剔除，考虑之前裁剪点云的时候缩小了100，这里放大200相当于放大100
                    // 而且由于是在中间位置放大，点云还是裁剪的，而且后期合并的时候也是合并的外围的，所以没有影响
                    auto extend_border_x = 200.0f + 100.0f;
                    left_carriage.border = {left_carriage.border.min_min,
                                            left_carriage.border.min_max,
                                            {left_carriage.border.max_min.x + extend_border_x,
                                             left_carriage.border.max_min.y, left_carriage.border.max_min.z},
                                            {left_carriage.border.max_max.x + extend_border_x,
                                             left_carriage.border.max_max.y, left_carriage.border.max_max.z}};
                    right_carriage.border = {
                            {right_carriage.border.min_min.x - extend_border_x, right_carriage.border.min_min.y,
                             right_carriage.border.min_min.z},
                            {right_carriage.border.min_max.x - extend_border_x, right_carriage.border.min_max.y,
                             right_carriage.border.min_max.z},
                            right_carriage.border.max_min,
                            right_carriage.border.max_max};

                    carriages.push_back(left_carriage);
                    carriages.push_back(right_carriage);
                }
                else {
                    // 获取车厢信息
                    carriages.push_back(getCarriage(cloud_in, cluster_cloud));
                }
            }
        }
        catch (std::exception &e) {
            std::cout << "异常是: " << e.what() << std::endl;
        }
    }

    return carriages;
}

Carriage CarScan::getCarriage(const pcl::PointCloud<pcl::PointXYZ>::Ptr &car_cloud,
                              const pcl::PointCloud<pcl::PointXYZ>::Ptr &carriage_cloud) {
    // 当前聚类的点云往往是车厢的点云，不包含钢卷
    // 拟合汽车车厢平面
    auto model_coefficients = fitCarBodyPlane(carriage_cloud);

    // 获取车厢的边界
    auto [car_range, car_plane_main_cloud] = getCarBodyEdge(carriage_cloud, config);

    // 将四个顶点保存
    saveCloudWhenDebug(createCloud(car_range.getPoints()), "06-车厢角点.pcd");

    // 裁剪车厢点云: 便于后续提取钢卷和鞍座点云
    auto cropped_car_body_cloud = cropCarCloud(car_cloud, car_range, config.crop_car_body_offset_x, config.crop_car_body_offset_y);

    return {car_range, model_coefficients, cropped_car_body_cloud};
}

bool CarScan::getCarriagesSingle(const pcl::PointCloud<pcl::PointXYZ>::Ptr& train_cloud, float x1, float y1, float x2, float y2, Carriage& output)
{
    return false;
}

std::vector<DetectResult> CarScan::formatDetectResult(const std::vector<DetectResult> &results) {
//    CAR_HEAD_POINT = 3, // 车厢头部点（X、Y地址最小的）
//    CAR_TAIL_POINT = 4, // 车厢尾部点（X、Y地址最大的）
    // 获取所有的车头的数据获取其中的最小值作为合并后的值
    auto min_head_x = std::numeric_limits<double>::max();
    auto min_head_y = std::numeric_limits<double>::max();
    auto min_head_z = std::numeric_limits<double>::max();
    for (auto &result: results) {
        if (result.data_type == DetectType::CAR_HEAD_POINT) {
            min_head_x = std::min(min_head_x, result.x);
            min_head_y = std::min(min_head_y, result.y);
            min_head_z = std::min(min_head_z, result.z);
        }
    }

    // 获取所有的车尾的数据获取其中的最大值作为合并后的值
    auto max_tail_x = std::numeric_limits<double>::min();
    auto max_tail_y = std::numeric_limits<double>::min();
    auto max_tail_z = std::numeric_limits<double>::min();
    for (auto &result: results) {
        if (result.data_type == DetectType::CAR_TAIL_POINT) {
            max_tail_x = std::max(max_tail_x, result.x);
            max_tail_y = std::max(max_tail_y, result.y);
            max_tail_z = std::max(max_tail_z, result.z);
        }
    }

    // 重新构造结果
    std::vector<DetectResult> new_results;
    float carriage_angle = 0.0f;
    for (auto &result: results) {
        if (result.data_type != DetectType::CAR_HEAD_POINT && result.data_type != DetectType::CAR_TAIL_POINT) {
            new_results.push_back(result);
        }
        else
            carriage_angle = result.axis;
    }

    // 添加新的结果
    new_results.push_back({ 0, DetectType::CAR_HEAD_POINT, min_head_x, min_head_y, min_head_z, 0, 0, carriage_angle });
    new_results.push_back({ 0, DetectType::CAR_TAIL_POINT, max_tail_x, max_tail_y, max_tail_z, 0, 0, carriage_angle });

    // 增加识别结果筛选
    // 去除距离太近的钢卷或者鞍座点
    // 不可能会距离太近
    std::vector<bool> remove_flags(new_results.size(), false);

    for (int i = 0; i < new_results.size() - 1; i++) {
        if (remove_flags[i]) continue;

        DetectResult& base = new_results[i];
        // 只处理钢卷和鞍座类型
        if (base.data_type != DetectType::COIL && base.data_type != DetectType::SADDLE) {
            continue;
        }
        // 判断钢卷是否合格
        if (base.data_type == DetectType::COIL)
        {
            if (base.width > config.coil_max_width || base.width < config.coil_min_width || base.diameter > config.coil_max_diameter || base.diameter < config.coil_min_diameter)
            {
                remove_flags[i] = true;
                continue;
            }
        }

        for (int j = i + 1; j < new_results.size(); j++) {
            if (remove_flags[j]) continue;

            DetectResult& comp = new_results[j];
            if (comp.data_type != DetectType::COIL && comp.data_type != DetectType::SADDLE) {
                continue;
            }

            // 计算X方向距离
            if (abs(base.x - comp.x) < config.saddle_length_min) {
                // 保留较低的点（Z值较小）
                if (base.z < comp.z) {
                    remove_flags[j] = true;
                }
                else {
                    remove_flags[i] = true;
                    break; // 当前base被标记删除，跳出内层循环
                }
            }
        }
    }

    // 从后往前删除标记的元素
    for (int i = new_results.size() - 1; i >= 0; i--) {
        if (remove_flags[i]) {
            new_results.erase(new_results.begin() + i);
        }
    }

    return new_results;
}

/**
 * 获取车厢的平面
 * @param car_body_cloud 输入的点云
 * @return 
 */
pcl::ModelCoefficients::Ptr
CarScan::fitCarBodyPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &car_body_cloud) {

    // 先裁剪部分，然后再拟合，避免边框对拟合产生影响， 但是后续裁剪才是使用原始的点云，避免导致车厢边框识别小于实际范围
    auto [min_point, max_point] = cloud::getMinMax(car_body_cloud);
    auto crop_car_plane_cloud = cloud::cropCloud(car_body_cloud,
                                                 min_point.x + config.crop_car_body_offset_x, max_point.x - config.crop_car_body_offset_x,
                                                 min_point.y + config.crop_car_body_offset_y, max_point.y - config.crop_car_body_offset_y,
                                                 min_point.z, min_point.z + 1000);
    if (crop_car_plane_cloud->empty()) {
        throw std::runtime_error("裁剪后的车厢点云为空");
    }
    Scan::saveCloudWhenDebug(crop_car_plane_cloud, "04-车厢底部识别裁剪点云.pcd");

    // 对点云进行拟合
    auto first_coefficients = cloud::fitPlane(crop_car_plane_cloud, 100);

    // 根据平面方程提取平面上下50mm的点云
    auto cloud_plane_up = plane::cropCloudByPlanDistance(first_coefficients, car_body_cloud, 0, 30);
    Scan::saveCloudWhenDebug(cloud_plane_up, "05-车厢底部最终平面.pcd");

    // 再次进行平面拟合，使得更接近真实的车厢平面，因为正常可能有鞍座第一个拟合出来的平面可能偏上
    auto second_coefficients = cloud::fitPlane(cloud_plane_up, 100);
    if (second_coefficients->values.empty()) {
        return first_coefficients;
    }

    return second_coefficients;
}

std::pair<PlaneQuadrilateral, pcl::PointCloud<pcl::PointXYZ>::Ptr>
CarScan::getCarBodyEdge(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &car_body_cloud,
        const ScanConfig &config) {

    // 直接使用车厢的点云，不根据平面进行裁剪，因为车厢平面拟合可能不准确，从而导致裁剪的范围不准确，可能导致裁剪的点云不完整，进一步导致车厢范围的4个点不准确
    // 聚类获取最大的点云: 再次聚类主要是为了去除车头的噪音（因为截取后这部分点云已经不和车头相连了）
    auto cluster_car_plane_cloud = cloud::maxEuclideanCluster(car_body_cloud, 200, 100);
    if (cluster_car_plane_cloud->empty()) throw std::runtime_error("汽车平面点云聚类后点云为空");
    Scan::saveCloudWhenDebug(cluster_car_plane_cloud, "06-车厢聚类点云.pcd");

    // 下采样: 降低点云的密度
    auto down_sample_car_plane_cloud = cloud::downSample(cluster_car_plane_cloud, 100);

    // sor降噪: 降噪的目的是为了避免车厢周边其他物体的干扰，导致后续的车厢范围的四个顶点的查询识别
    auto sor_down_sample_car_plane_cloud = cloud::sorFilter(down_sample_car_plane_cloud,
                                                            config.carriage_sor_mean_k,
                                                            config.carriage_sor_stddev_mul_thresh);

    // 获取下采样后的边界点: 主要是为了获取车厢的边界点
    auto car_body_points = cloud::findXyAnglePoints(sor_down_sample_car_plane_cloud);

    // 再次拟合平面
    auto new_coefficients = cloud::fitPlane(sor_down_sample_car_plane_cloud, 100);
    if (new_coefficients->values.empty()) {
        saveCloudToPcd(sor_down_sample_car_plane_cloud, "sor_down_sample_car_plane_cloud.pcd");
        throw std::runtime_error("拟合平面失败");
    }

    // 生成四个顶点
    float a = new_coefficients->values[0];
    float b = new_coefficients->values[1];
    float c = new_coefficients->values[2];
    float d = new_coefficients->values[3];

    // 重新计算Z
    for (auto &point: car_body_points) {
        point.z = (-d - a * point.x - b * point.y) / c;
    }
    return {{car_body_points[0], car_body_points[1], car_body_points[2], car_body_points[3]},
            sor_down_sample_car_plane_cloud};
}

/**
 * @brief 裁剪车厢的点云，用于后续的处理
 * @param cloud_in 输入点云，车厢点云 
 * @param car_min_min 车厢的4个顶点 
 * @param car_min_max 车厢的4个顶点 
 * @param car_max_min 车厢的4个顶点
 * @param car_max_max 车厢的4个顶点 
 * @return 
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr
CarScan::cropCarCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, PlaneQuadrilateral car_range,
                      float x_offset, float y_offset) {
    // 定义输出点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_car_body_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 将四个点分别往内缩减指定的距离 TODO: 待修改函数
    auto new_range = car_range.scaleDistanceXy(-x_offset);

    // 获取最大最小X、Y: TODO: 这里其实还是按照矩形来处理，但是四边形可能是倾斜或者不规整的
    auto [min_x, max_x] = new_range.xRange();
    auto [min_y, max_y] = new_range.yRange();

    // 裁剪点云
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(min_x, max_x);
    pass.filter(*cropped_car_body_cloud);

//    pass.setInputCloud(cropped_car_body_cloud);
//    pass.setFilterFieldName("y");
//    pass.setFilterLimits(min_y, max_y);
//    pass.filter(*cropped_car_body_cloud);

    return cropped_car_body_cloud;
}

/**
 * 识别鞍座
 * @param saddles_cloud 包含所有鞍座的点云
 * @param coils_result 钢卷的识别结果
 * @return 鞍座识别结果
 */
std::vector<DetectResult>
CarScan::detectSaddles(const pcl::PointCloud<pcl::PointXYZ>::Ptr &saddles_cloud,
                       const std::vector<DetectResult> &coils_result, const Carriage &carriage) {

    // 遍历钢卷结果，根据X剔除鞍座点云中对应范围的点云
    auto cropped_saddles_cloud = cropSaddleCloudByCoils(saddles_cloud, coils_result);
    saveCloudWhenDebug(cropped_saddles_cloud, "10-鞍座点云.pcd");

    // 如果裁剪后点云已经为空，则直接返回
    if (cropped_saddles_cloud->empty()) {
        return {};
    }

    // 对点云执行sor降噪
    cropped_saddles_cloud = cloud::sorFilter(cropped_saddles_cloud,
                                             config.saddles_sor_mean_k,
                                             config.saddles_sor_stddev_mul_thresh);
    saveCloudWhenDebug(cropped_saddles_cloud, "11-降噪后鞍座点云.pcd");

    // 聚类获取鞍座
    auto saddle_clusters = cloud::euclideanCluster(cropped_saddles_cloud,
                                                   config.saddle_cluster_tolerance,
                                                   config.saddle_cluster_min_size);

    // 计算聚类X方向大小的平均值
    auto average_x_size = std::accumulate(saddle_clusters.begin(), saddle_clusters.end(), 0.0,
                                          [](double sum, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
                                              auto [min_point, max_point] = cloud::getMinMax(cloud);
                                              return sum + max_point.x - min_point.x;
                                          }) / static_cast<double>(saddle_clusters.size());
    // 计算Z方向的大小的平均值
    auto average_z_size = std::accumulate(saddle_clusters.begin(), saddle_clusters.end(), 0.0,
                                          [](double sum, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
                                              auto [min_point, max_point] = cloud::getMinMax(cloud);
                                              return sum + max_point.z - min_point.z;
                                          }) / static_cast<double>(saddle_clusters.size());

    // 目前从实际的数据看
    // 皮垫：  x_size 约800左右   z_size 270左右
    // 纯鞍座: x_size 约450左右   z_size 155左右
    // 草垫：  x_size 约366      z_size 100左右
    // 由于纯鞍座的和草垫的x的大小类似，所以加上z大小的限制

    if (average_x_size < 600 && average_z_size < 100) {
        std::cout << "鞍座类型为草垫" << std::endl;
        return detectStrawSaddles(cropped_saddles_cloud, carriage);
    } else {
        std::cout << "鞍座类型为普通鞍座" << std::endl;
        return detectNormalSaddles(saddle_clusters, carriage);
    }
}

/**
 * 根据钢卷的识别结果，裁剪鞍座点云
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr
CarScan::cropSaddleCloudByCoils(const pcl::PointCloud<pcl::PointXYZ>::Ptr &saddles_cloud,
                                const std::vector<DetectResult> &coils_result) {
    // 首先将coils的识别结果根据X从小到大排序
    std::vector<DetectResult> sorted_coils_result = coils_result;
    std::sort(sorted_coils_result.begin(), sorted_coils_result.end(),
              [](const DetectResult &a, const DetectResult &b) {
                  return a.x < b.x;
              });

    // 遍历钢卷结果，根据X剔除鞍座点云中对应范围的点云
    // 如果前一个钢卷的最大X和后一个钢卷的最小X之间的距离小于300mm，则认为是相邻的卷，将这两个卷之间的点云都剔除
    auto cropped_saddles_cloud = saddles_cloud;
    auto last_coil_max_x = std::numeric_limits<float>::min();
    for (auto &coil_result: sorted_coils_result) {
        // 获取钢卷的X范围
        auto min_x = static_cast<float>(coil_result.x - coil_result.diameter / 2.0);
        auto max_x = static_cast<float>(coil_result.x + coil_result.diameter / 2.0);

        // 如果前一个钢卷的最大X和后一个钢卷的最小X之间的距离小于700mm，则认为是相邻的卷，将这两个卷之间的点云都剔除
        if (min_x - last_coil_max_x < 700) {
            cropped_saddles_cloud = cloud::cropCloudX(cropped_saddles_cloud, last_coil_max_x, max_x, true);
        } else {
            cropped_saddles_cloud = cloud::cropCloudX(cropped_saddles_cloud, min_x - 50, max_x + 50, true);
        }

        last_coil_max_x = max_x;
    }

    return cropped_saddles_cloud;
}

std::vector<DetectResult>
CarScan::detectNormalSaddles(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &saddle_clusters,
                             const Carriage &carriage) {
    // 合并鞍座: 由于鞍座可能是断开的，所以需要合并
    auto merged_saddle_clusters = cloud::mergeClouds(saddle_clusters);
    if (merged_saddle_clusters->empty()) {
        return {};
    }
    saveCloudWhenDebug(merged_saddle_clusters, "11-合并后鞍座点云.pcd");

    // 遍历聚类进行识别
    std::vector<DetectResult> detect_results;

    // 获取大小，判断X的长度是否大于1500，如果是则截取识别
    int idx = 0;
    while (true) {
        auto [min_point, max_point] = cloud::getMinMax(merged_saddle_clusters);

        // 截取2000的点云
        auto cropped_saddle_cluster = cloud::cropCloud(merged_saddle_clusters,
                                                       min_point.x, min_point.x + 2000,
                                                       min_point.y, max_point.y,
//                                                       min_point.y + 50, max_point.y - 50,    // Y方向裁掉一部分不见的能将皮垫都裁剪完
                                                       min_point.z, max_point.z);

        auto cropped_saddle_cluster_sor = cloud::sorFilter(cropped_saddle_cluster,
                                                           config.saddle_sor_mean_k,
                                                           config.saddle_sor_stddev_mul_thresh);

        // 获取最大最小
//        auto min_z_point = cloud::getMinZPoint(cropped_saddle_cluster);   // 导致这里获取Z的最低点可能不准确
        auto min_z_point = findFirstMinZPoint(cropped_saddle_cluster_sor);

        // 截取左侧
        auto left_saddle_cluster = cloud::cropCloudX(cropped_saddle_cluster_sor, min_point.x,
                                                     min_z_point.x);
        auto right_saddle_cluster = cloud::cropCloudX(cropped_saddle_cluster_sor, min_z_point.x,
                                                      std::numeric_limits<float>::max());

        // 如果左侧或者右侧为空，则继续，抛弃当前的点云
        if (left_saddle_cluster->empty() || right_saddle_cluster->empty()) {
            // 获取两部分中最大的X
            auto [left_min_point, left_max_point] = cloud::getMinMax(left_saddle_cluster);
            auto [right_min_point, right_max_point] = cloud::getMinMax(right_saddle_cluster);
            auto max_x = std::max(left_max_point.x, right_max_point.x);

            merged_saddle_clusters = cloud::cropCloudX(merged_saddle_clusters,
                                                       max_x + 1,  // +1 是避免正好保留的max_x,当只有这一个点的时候，可能导致这个点永远无法被删除，导致死循环
                                                       std::numeric_limits<float>::max());

            // 重新计算大小
            auto [size_x, size_y, size_z] = cloud::getCloudSize(merged_saddle_clusters);
            if (size_x < 1000)
                break;

            continue;
        }

        // 先将点云压缩到Y方向，使得Y方向的点都到一起，使得隔开的两个鞍座突出可以合并到一起计算，避免聚类失败
        auto compressed_left_saddle_cluster = cloud::compressY(left_saddle_cluster);
        auto compressed_right_saddle_cluster = cloud::compressY(right_saddle_cluster);
        // 聚类左侧和右侧，分别取其左侧结束地址最大的聚类和右侧开始地址最小的聚类
        auto left_saddle_clusters = cloud::euclideanCluster(compressed_left_saddle_cluster,
                                                            config.saddle_cluster_tolerance,
                                                            40); // TODO: 待提取成参数
//                                                            config.saddle_cluster_min_size);
        auto right_saddle_clusters = cloud::euclideanCluster(compressed_right_saddle_cluster,
                                                             config.saddle_cluster_tolerance,
                                                             40); // TODO: 待提取成参数
//                                                             config.saddle_cluster_min_size);

        if (left_saddle_clusters.empty() || right_saddle_clusters.empty()) {
            auto [left_min_point, left_max_point] = cloud::getMinMax(left_saddle_cluster);
            auto [right_min_point, right_max_point] = cloud::getMinMax(right_saddle_cluster);
            auto max_x = std::max(left_max_point.x, right_max_point.x);

            merged_saddle_clusters = cloud::cropCloudX(merged_saddle_clusters,
                                                       max_x + 1,  // +1 是避免正好保留的max_x,当只有这一个点的时候，可能导致这个点永远无法被删除，导致死循环
                                                       std::numeric_limits<float>::max());

            // 重新计算大小
            auto [size_x, size_y, size_z] = cloud::getCloudSize(merged_saddle_clusters);
            if (size_x < 1000)
                break;

            continue;
        }

        // 排序获取最大地址X最大的聚类
        auto left_best_saddle_cluster = *std::max_element(left_saddle_clusters.begin(), left_saddle_clusters.end(),
                                                          [](const pcl::PointCloud<pcl::PointXYZ>::Ptr &a,
                                                             const pcl::PointCloud<pcl::PointXYZ>::Ptr &b) {
                                                              auto [min_point_a, max_point_a] = cloud::getMinMax(a);
                                                              auto [min_point_b, max_point_b] = cloud::getMinMax(b);
                                                              return max_point_a.x < max_point_b.x;
                                                          });


        // 排序获取最小地址X最小的聚类
        auto right_best_saddle_cluster = *std::min_element(right_saddle_clusters.begin(), right_saddle_clusters.end(),
                                                           [&min_z_point](const pcl::PointCloud<pcl::PointXYZ>::Ptr &a,
                                                              const pcl::PointCloud<pcl::PointXYZ>::Ptr &b) {
                                                               auto [min_point_a, max_point_a] = cloud::getMinMax(a);
                                                               auto [min_point_b, max_point_b] = cloud::getMinMax(b);
                                                               
                                                                if (min_point_a.x < min_point_b.x) {
                                                                    // 排除掉起始离中间位置很近，而且点云数量很少的点云
                                                                    if (std::abs(min_point_a.x - min_z_point.x) < 100 && a->size() < 100) {
                                                                        return false;
                                                                    }

                                                                    return true;
                                                                }
                                                               return false;
                                                           });

        // 保存鞍座点云
        auto merge_saddle_cloud = cloud::mergeClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>{
                left_best_saddle_cluster, right_best_saddle_cluster
        });

        // 分别获取左右侧的最大点
        // TODO: 两边的最大点可能并不完整，导致位置偏差
        // TODO: 最大值可能是旁边鞍座的点，可能导致库位坐标进一步偏移
        auto left_max_point = cloud::getMaxZPoint(left_best_saddle_cluster);
        auto right_max_point = cloud::getMaxZPoint(right_best_saddle_cluster);

        // 判断是否合理： 1 最低点距离两侧的最高点距离在600mm以上 2 最低点和最高点的高度差在100mm
        // 纯鞍座的两个最高点x方向大小只有约800左右（部分车型）
        if (abs(left_max_point.x - right_max_point.x) > 800 &&
            left_max_point.z - min_z_point.z > 80 && right_max_point.z - min_z_point.z > 80) {

            // 根据最大点截取点云
            auto detect_one_saddle_cloud = cloud::cropCloudX(cropped_saddle_cluster_sor, left_max_point.x,
                                                             right_max_point.x);

            // 获取中心点
            auto [one_min_point, one_max_point] = cloud::getMinMax(detect_one_saddle_cloud);

            // 计算X和Y
            auto saddle_x = (one_min_point.x + one_max_point.x) / 2;
            // Y坐标取天车中心Y
            auto saddle_y = carriage.border.getCenterY(saddle_x);
//            auto saddle_y = (one_min_point.y + one_max_point.y) / 2;

            // 截取点左右X的200范围内的点云，判断点云数量，如果过少则认为无效: 见用例20240520182525034-ba.pcd
            // 截取左侧和右侧的点云
            auto left_saddle_cloud = cloud::cropCloudX(cropped_saddle_cluster_sor, min_point.x, saddle_x);
            auto right_saddle_cloud = cloud::cropCloudX(cropped_saddle_cluster_sor, saddle_x, max_point.x);

            // 判断两侧点云的x方向的长度
            auto [left_size_x, left_size_y, left_size_z] = cloud::getCloudSize(left_saddle_cloud);
            auto [right_size_x, right_size_y, right_size_z] = cloud::getCloudSize(right_saddle_cloud);

            // 如果两侧的点云长度都小于200mm，则认为无效（取反：任意侧点云长度大于200则认为有效）
            if ((left_size_x > 200 || right_size_x > 200) && detect_one_saddle_cloud->size() > 500) {
                auto nearby_saddle_cloud = cloud::cropCloudX(cropped_saddle_cluster_sor, saddle_x - 200,
                                                             saddle_x + 200);
                // Z根据平面方程计算
                auto saddle_z = plane::calcZ(carriage.plane, saddle_x, saddle_y);

                // 计算角度
                // double angle = computeMinBoundingBoxAnglePCA(detect_one_saddle_cloud);
                double angle = computeUnwindingAngleFromSaddle(detect_one_saddle_cloud);

                double width = one_max_point.x - one_min_point.x;
                double delimiter = one_max_point.y - one_min_point.y;

                // 记录识别结果
                saveCloudWhenDebug(detect_one_saddle_cloud, "12-第" + std::to_string(idx) + "个鞍座.pcd");
                idx++;
                std::cout << "添加第" << idx << "次鞍座识别结果，点云数量:" << detect_one_saddle_cloud->size() << "    左侧高度:" << left_size_z << "    右侧高度:" << right_size_z << "    左侧长度:" << left_size_y << "    右侧长度" << right_size_y << "    角度: " << angle << "    宽度: " << width << "    直径: " << delimiter << std::endl;
                detect_results.push_back({0, DetectType::SADDLE, saddle_x, saddle_y, saddle_z, width, delimiter, formatAngle(angle, 0)});
            }
        }


        // 截取剩余的点云
        auto saddle_max_x = std::max(left_max_point.x, right_max_point.x) + 60;
        merged_saddle_clusters = cloud::cropCloudX(merged_saddle_clusters,
                                                   saddle_max_x + 1, // +1 是避免正好保留的max_x,当只有这一个点的时候，可能导致这个点永远无法被删除，导致死循环
                                                   std::numeric_limits<float>::max());

        // 重新计算大小
        auto [size_x, size_y, size_z] = cloud::getCloudSize(merged_saddle_clusters);
        if (size_x < 900)
            break;
    }

    return detect_results;
}

std::vector<DetectResult>
CarScan::detectStrawSaddles(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cropped_saddles_cloud,
                            const Carriage &carriage) {

    // 需要重新聚类是因为草垫单个聚类的数据小，不能使用默认的100
    auto saddle_clusters = cloud::euclideanCluster(cropped_saddles_cloud,
                                                   config.saddle_cluster_tolerance,
                                                   30);  // TODO: 参数待设置

    // 合并鞍座
    auto merged_saddle_clusters = cloud::mergeClouds(saddle_clusters);
    if (merged_saddle_clusters->empty()) {
        return {};
    }
    saveCloudWhenDebug(merged_saddle_clusters, "merged_saddle_clusters.pcd");

    // 计算每个聚类的中心X值
    std::vector<float> cluster_centers;
    for (auto &cluster: saddle_clusters) {
        auto [min_point, max_point] = cloud::getMinMax(cluster);
        cluster_centers.push_back((min_point.x + max_point.x) / 2);
    }

    // 按照X值从小到大排序
    std::sort(cluster_centers.begin(), cluster_centers.end());

    // 判断是否有距离很近的中心点，如果有则合并，小于300mm则合并
    for (int i = 0; i < cluster_centers.size() - 1; i++) {
        if (cluster_centers[i + 1] - cluster_centers[i] < 300) {
            cluster_centers[i] = (cluster_centers[i] + cluster_centers[i + 1]) / 2;
            cluster_centers.erase(cluster_centers.begin() + i + 1);
            i--;
        }
    }

    // 取相邻两个点的中心点作为鞍座的中心点
    std::vector<DetectResult> detect_results;
    for (int i = 0; i < cluster_centers.size() - 1; i++) {
        auto saddle_x = (cluster_centers[i] + cluster_centers[i + 1]) / 2;
        // Y取车厢的中心点
        auto saddle_y = carriage.border.getCenterY(saddle_x);
        auto saddle_z = plane::calcZ(carriage.plane, saddle_x, saddle_y);
        detect_results.push_back({0, DetectType::SADDLE, saddle_x, saddle_y, saddle_z, 0, 0, 0});
    }

    return detect_results;
}