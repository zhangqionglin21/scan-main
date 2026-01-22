#ifndef SCAN_TRAIN_SCAN_H
#define SCAN_TRAIN_SCAN_H

#include "scan.h"


/**
 * 火车扫描
 */
class TrainScan : public Scan {

public:
    TrainScan();

    /**
     * @brief 配置默认配置
     * @return 
     */
    ScanConfig getDefaultConfig() override {
        auto config = ScanConfig();

        config.car_height = 2500;
        config.cloud_crop_x_min = 120 * 1000;
        config.cloud_crop_y_min = 2.5 * 1000;
        config.cloud_crop_z_min = 500;
        config.cloud_crop_x_max = 140 * 1000;
        config.cloud_crop_y_max = 7 * 1000;
        config.cloud_crop_z_max = 4000;
        
        config.crop_saddle_offset_y = 100;
        config.crop_car_body_offset_x = 100;
        config.crop_saddle_offset_z_max = 600;
        
        config.crop_saddle_offset_z_min = 150;
        config.crop_coil_offset_z_min = 700;
        config.min_saddle_interval = 1000;
        config.saddle_cluster_min_size = 15;
        config.saddle_sor_mean_k = 10;
        config.coil_cluster_min_size = 250;

        return config;
    };

public:
    bool getCarriagesSingle(const pcl::PointCloud<pcl::PointXYZ>::Ptr& train_cloud, float x1, float y1, float x2, float y2, Carriage& output)override;
    Carriage getCarriage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& car_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& carriage_cloud);
    //Carriage getCarriagesSingle(const pcl::PointCloud<pcl::PointXYZ>::Ptr& train_cloud, float x1, float y1, float x2, float y2);
    //Carriage getCarriagesSingle(const pcl::PointCloud<pcl::PointXYZ>::Ptr& train_cloud);
    /**
     * @brief 从整体的点云中提取出车厢信息，包含车厢部分的点云（一般不包含围栏），车厢的边界信息
     * @param cloud_in 
     * @return 
     */
    std::vector<Carriage> getCarriages(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in) override;
  
    /**
     * 判断是否是完整的车厢
     * @param car_border_cloud 单个车厢边框上部点云
     * @param scan_config 
     * @return 
     */
    static bool
    isFullCarriage(const pcl::PointCloud<pcl::PointXYZ>::Ptr &car_border_cloud, const ScanConfig &scan_config);

    /**
     * 计算车厢边界
     * @param car_border_cloud 
     * @param scan_config 
     * @return 
     */
    static MinMaxPoint
    calcCarriageBorder(const pcl::PointCloud<pcl::PointXYZ>::Ptr &car_border_cloud, const ScanConfig &scan_config);


    /**
     * @brief 格式化输出点云
     * @param cloud: 输入点云
     * @param cloud_out: 输出用的点云
     * @param results: 识别的结果，用于辅助裁剪
     */
    void
    formatOutputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ> *cloud_out,
                      const std::vector<DetectResult> &results) override;
};


// TODO: 后续通过配置文件读取
const std::vector<Chassis> TRAIN_CHASSIS = {
        // Index: 0 => A1: 1*2, X:1580 Y: 0
        {
                {
                        {0, 0, 0},
                        {1580, 0,   0}
                },
                {
                        {790, 0,     0}
                }
        },
        // Index: 1 => B2 1*3, X: 1600 Y: 0
        {
                {
                        {0, 0, 0},
                        {1600, 0,   0},
                        {3200, 0,    0}
                },
                {
                        {800, 0,     0},
                        {2400, 0,     0}
                }
        },
        // Index: 2 => C1 4*2 X: 1280 Y: 700
        {
                {
                        {0, 0, 0},
                        {0,    700, 0},
                        {0,    1400, 0},
                        {0,    2100, 0},
                        {1280, 0, 0},
                        {1280, 700, 0},
                        {1280, 1400, 0},
                        {1280, 2100, 0}
                },
                {
                        {640, 1050,  0},
                }
        },
        // Index: 3 =>  C2 4*3 X: 1380 Y 690
        {
                {
                        {0, 0, 0},
                        {0,    690, 0},
                        {0,    1380, 0},
                        {0,    2070, 0},
                        {1380, 0, 0},
                        {1380, 690, 0},
                        {1380, 1380, 0},
                        {1380, 2070, 0},
                        {2760, 0, 0},
                        {2760, 690, 0},
                        {2760, 1380, 0},
                        {2760, 2070, 0},
                },
                {
                        {690, 1035,  0},
                        {2070, 1035,  0}
                }
        },
        // Index: 4 =>  C3 4*4 X: 1180 Y: 两侧660 中间 760
        {
                {
                        {0, 0, 0},
                        {0,    660, 0},
                        {0,    1420, 0},
                        {0,    2080, 0},
                        {1180, 0, 0},
                        {1180, 660, 0},
                        {1180, 1420, 0},
                        {1180, 2080, 0},
                        {2360, 0, 0},
                        {2360, 660, 0},
                        {2360, 1420, 0},
                        {2360, 2080, 0},
                        {3540, 0, 0},
                        {3540, 660, 0},
                        {3540, 1420, 0},
                        {3540, 2080, 0}
                },
                {
                        {590, 1040,  0},
                        {1770, 1040,  0},
                        {2950, 1040,  0}
                }
        },
        // Index: 5 =>  Q3 2*5 X: 1300, Y: 635
        {
                {
                        {0, 0, 0},
                        {0,    635, 0},
                        {1300, 0,    0},
                        {1300, 635,  0},
                        {2600, 0, 0},
                        {2600, 635, 0},
                        {3900, 0,    0},
                        {3900, 635,  0},
                        {5200, 0, 0},
                        {5200, 635, 0}
                },
                {
                        {650, 317.5, 0},
                        {1950, 317.5, 0},
                        {3250, 317.5, 0},
                        {4550, 317.5, 0}
                }
        },
        // Index: 6 =>  T4: 2*5  X:1300 Y: 610
        {
                {
                        {0, 0, 0},
                        {0,    610, 0},
                        {1300, 0,    0},
                        {1300, 610,  0},
                        {2600, 0, 0},
                        {2600, 610, 0},
                        {3900, 0,    0},
                        {3900, 610,  0},
                        {5200, 0, 0},
                        {5200, 610, 0}
                },
                {
                        {650, 305,   0},
                        {1950, 305,   0},
                        {3250, 305,   0},
                        {4550, 305,   0}
                }
        },
        // Index: 7 =>  D3P: 4*3  
        {
                {
                        {0, 0, 0},
                        {0, 530, 0},
                        {0, 1230, 0},
                        {0, 1760, 0},
                        {1150, 0, 0},
                        {1150, 530, 0},
                        {1150, 1230, 0},
                        {1150, 1760, 0},
                        {2350, 0, 0},
                        {2350, 530, 0},
                        {2350, 1230, 0},
                        {2350, 1760, 0},
                        {3500, 0, 0},
                        {3500, 530, 0},
                        {3500, 1230, 0},
                        {3500, 1760, 0}
                },
                {
                        {575, 880,  0},
                        {1750, 880, 0},
                        {2925, 880, 0}
                }
        },
    
};

#endif //SCAN_TRAIN_SCAN_H
