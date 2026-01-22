#include "agv_scan.h"
#include "extensions.h"

AgvScan::AgvScan() : CarScan(getDefaultConfig()) {
    chassis = AGV_CHASSIS;
}


/**
 * 识别鞍座
 * @param saddles_cloud 包含所有鞍座的点云
 * @param config 配置信息
 * @return 
 */
std::vector<DetectResult>
AgvScan::detectSaddles(const pcl::PointCloud<pcl::PointXYZ>::Ptr &saddles_cloud,
                       const std::vector<DetectResult> &coils_result, const Carriage &carriage) {
    // TODO: 识别鞍座
    return Scan::detectSaddles(saddles_cloud, coils_result, carriage);
}

bool AgvScan::getCarriagesSingle(const pcl::PointCloud<pcl::PointXYZ>::Ptr& train_cloud, float x1, float y1, float x2, float y2, Carriage& output)
{
    return false;
}
/**
 * @brief 从整体的点云中提取出车厢信息，包含车厢部分的点云（一般不包含围栏），车厢的边界信息
 * @param cloud_in 输入的点云
 * @return 
 */
std::vector<Carriage> AgvScan::getCarriages(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in) {
    std::cout << "使用AgvScan::getCarriages检测" << std::endl;
    auto carriages = std::vector<Carriage>();
    auto cluster_clouds = cloud::euclideanCluster(cloud_in, 100, 10);

    //int cout = 0;
    for (auto &cluster_cloud: cluster_clouds) {
        try {
            // 获取点云的大小
            auto [min_point, max_point] = cloud::getMinMax(cluster_cloud);
            auto [x_size, y_size, z_size] = cloud::getXyzSize(min_point, max_point);

            //std::string tempstr = "test111_" + std::to_string(cout) + ".pcd";
            //cout += 1;
            //saveCloudWhenDebug(cluster_cloud, tempstr);
            // 判断是否在车身长度、宽度的范围内
            if (x_size > (config.car_length * 0.8) && y_size > (config.car_width * 0.8) &&
                max_point.z > (config.car_height * 0.8)) {

                // 获取车厢信息
                carriages.push_back(getCarriage(cloud_in, cluster_cloud));
                break;
            }
        }
        catch (std::exception &e) {
            std::cout << "异常是: " << e.what() << std::endl;
        }
    }

    return carriages;
}
