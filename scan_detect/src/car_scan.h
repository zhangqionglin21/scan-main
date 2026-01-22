#ifndef SCAN_CAR_SCAN_H
#define SCAN_CAR_SCAN_H

#include "scan.h"

class CarScan : public Scan {
public:
    CarScan();

    CarScan(const ScanConfig &config);

    /**
     * @brief 配置默认配置
     * @return 
     */
    ScanConfig getDefaultConfig() override {
        auto config = ScanConfig();

        /*config.car_height = 2.0 * 1000;
        config.car_length = 10 * 1000;
        config.car_width = 2.4 * 1000;
        config.cloud_crop_x_min = 120 * 1000;
        config.cloud_crop_y_min = 2.5 * 1000;
        config.cloud_crop_z_min = 500;
        config.cloud_crop_x_max = 140 * 1000;
        config.cloud_crop_y_max = 7 * 1000;
        config.cloud_crop_z_max = 4000;
        config.crop_car_body_offset_x = 100;
        config.crop_saddle_offset_z_min = 50;
        config.crop_coil_offset_z_min = 600;
        config.saddle_cluster_min_size = 50;*/

        config.car_height = 1.0 * 1000;
        config.car_length = 5 * 1000;
        config.car_width = 2.4 * 1000;
        config.cloud_crop_x_min = 50000;
        config.cloud_crop_y_min = 1000;
        config.cloud_crop_z_min = 1400;
        config.cloud_crop_x_max = 70000;
        config.cloud_crop_y_max = 20000;
        config.cloud_crop_z_max = 4000;
        config.crop_car_body_offset_x = 100;
        config.crop_saddle_offset_z_min = 50;
        config.crop_coil_offset_z_min = 600;
        config.saddle_cluster_min_size = 50;

        return config;
    };

public:
    /**
     * @brief 从整体的点云中提取出车厢信息，包含车厢部分的点云（一般不包含围栏），车厢的边界信息
     * @param cloud_in 输入的点云
     * @return 
     */
    std::vector<Carriage> getCarriages(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in) override;


    /**
     * @brief 格式化检测结果，正常不做任何处理，主要是根据特殊情况进行处理, 比如汽车的高低差的车厢
     * @param results 检测的结果
     * @return 格式化后的结果
     */
    std::vector<DetectResult> formatDetectResult(const std::vector<DetectResult> &results) override;
    
    /**
     * @brief 从单个车厢点云中获取到车厢的信息，要求车厢的点云已经是单个车厢的点云
     */
    Carriage getCarriage(const pcl::PointCloud<pcl::PointXYZ>::Ptr &car_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr &carriage_cloud) override;

    bool getCarriagesSingle(const pcl::PointCloud<pcl::PointXYZ>::Ptr& train_cloud, float x1, float y1, float x2, float y2, Carriage& output);

    //Carriage getCarriagesSingle(const pcl::PointCloud<pcl::PointXYZ>::Ptr& train_cloud, float x1, float y1, float x2, float y2);
    /**
     * 获取车厢的平面方程
     * @param car_body_cloud 输入的点云
     * @return coefficients 输出的平面系数, 以及对应匹配上的点云
     */
    pcl::ModelCoefficients::Ptr
    fitCarBodyPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &car_body_cloud);


    /**
     * 根据天车点云以及车厢的平面方程，获取车厢的四个顶点
     * @param car_body_cloud 已经提取的车厢的点云
     * @param config 配置信息 
     * @return 车厢的边界范围，车厢平面点云
     */
    std::pair<PlaneQuadrilateral, pcl::PointCloud<pcl::PointXYZ>::Ptr>
    getCarBodyEdge(const pcl::PointCloud<pcl::PointXYZ>::Ptr &car_body_cloud,
                   const ScanConfig &config);


    /**
     * 裁剪车厢的点云，主要是根据车厢的四个顶点进行裁剪, 用于剔除围栏等干扰
     * @param cloud_in 
     * @param cloud_out 
     * @param car_min_min 
     * @param car_min_max 
     * @param car_max_min 
     * @param car_max_max 
     * @return 
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr cropCarCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                                                            PlaneQuadrilateral car_range,
                                                            float x_offset = 500, float y_offset = 300);

    /**
     * 识别鞍座
     * @param saddles_cloud 包含所有鞍座的点云
     * @param coils_result 钢卷的识别结果
     * @return 鞍座识别结果 
     */
    std::vector<DetectResult>
    detectSaddles(const pcl::PointCloud<pcl::PointXYZ>::Ptr &saddles_cloud,
                  const std::vector<DetectResult> &coils_result, const Carriage &carriage) override;


    /**
     * @brief 识别正常的鞍座
     * @param saddle_clusters 鞍座的点云
     * @return 鞍座识别结果
     */
    std::vector<DetectResult>
    detectNormalSaddles(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &saddle_clusters, const Carriage &carriage);

    /**
     * @brief 识别草垫
     * @param saddle_clusters 鞍座的点云
     * @return 鞍座识别结果
     */
    std::vector<DetectResult>
    detectStrawSaddles(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cropped_saddles_cloud,
                       const Carriage &carriage);
    
    /**
     * @brief 根据钢卷的识别结果，裁剪鞍座的点云
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    cropSaddleCloudByCoils(const pcl::PointCloud<pcl::PointXYZ>::Ptr &saddles_cloud, const std::vector<DetectResult> &coils_result);
};


#endif //SCAN_CAR_SCAN_H
