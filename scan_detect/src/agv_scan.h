#ifndef SCAN_AGV_SCAN_H
#define SCAN_AGV_SCAN_H


#include "car_scan.h"

class AgvScan : public CarScan {
public:
    AgvScan();

    /**
     * 识别鞍座
     * @param saddles_cloud 包含所有鞍座的点云
     * @param config 配置信息
     * @return 
     */
    std::vector<DetectResult>
    detectSaddles(const pcl::PointCloud<pcl::PointXYZ>::Ptr &saddles_cloud, const std::vector<DetectResult> &coils_result, const Carriage &carriage) override;

    


    /**
     * @brief 从整体的点云中提取出车厢信息，包含车厢部分的点云（一般不包含围栏），车厢的边界信息
     * @param cloud_in 输入的点云
     * @return 
     */
    std::vector<Carriage> getCarriages(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in) override;
    bool getCarriagesSingle(const pcl::PointCloud<pcl::PointXYZ>::Ptr& train_cloud, float x1, float y1, float x2, float y2, Carriage& output)override;
    
    /**
     * @brief 配置AGV的默认配置
     * @return 
     */
    ScanConfig getDefaultConfig() override {
        auto config = ScanConfig();

        config.car_length = 9.5 * 1000;
        config.car_width = 2.6 * 1000;
        config.car_height = 1.4 * 1000;
        config.cloud_crop_x_min = -8 * 1000;
        config.cloud_crop_x_max = 8 * 1000;
        config.cloud_crop_y_min = -1.8 * 1000;
        config.cloud_crop_y_max = 1.8 * 1000;
        config.cloud_crop_z_min = 500;
        config.cloud_crop_z_max = 4000;
        config.crop_saddle_offset_y = 200;  // 将边上是若隐若现的鞍座点云裁剪掉
        config.crop_car_body_offset_y = 0;
        config.saddle_cluster_tolerance = 100;
        config.saddle_cluster_min_size = 50;

        return config;
    };
};

// TODO: 后续通过配置文件读取
const std::vector<Chassis> AGV_CHASSIS = {
        // 三个鞍座的情况
        {
                {
                        {0, 0, 0},
                        {1430, 0, 0},
                        // 875间隔
                        {2305, 0, 0},
                        {3735, 0, 0},
                        // 875间隔
                        {4610, 0, 0},
                        {6040, 0, 0},
                },
                {
                        {715, 0, 0},
                        {3020, 0, 0},
                        {5325, 0, 0}

                }
        },
        // 两个相连鞍座的情况
        {
                {
                        {0, 0, 0},
                        {1430, 0, 0},
                        // 875间隔
                        {2305, 0, 0},
                        {3735, 0, 0},
                },
                {
                        {715, 0, 0},
                        {3020, 0, 0}

                }
        },
        // 中间空了一个鞍座的情况
        {
                {
                        {0, 0, 0},
                        {1430, 0, 0},
                        // 875间隔
                        {4610, 0, 0},
                        {6040, 0, 0},
                },
                {
                        {715, 0, 0},
                        {5325, 0, 0}

                }
        },
        // 单个鞍座的情况
        {
                {
                        {0, 0, 0},
                        {1430, 0, 0}
                },
                {
                        {715, 0, 0}

                }
        }
};


#endif //SCAN_AGV_SCAN_H
