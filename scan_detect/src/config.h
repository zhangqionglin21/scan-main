#ifndef SCAN_CONFIG_H
#define SCAN_CONFIG_H

#include "types.h"


/**
 * 扫描配置信息
 */
struct ScanConfig {
    // 是否开启日志
    bool enable_log = false;
    // 是否开启点云日志
    bool enable_debug_pcd = false;

    // 车头的方向，分别支持x、y、-x、-y
    // x表示朝向x大地址方向，-x表示朝向x小地址方向，其他类推
    bool auto_detect_car_header = true;
    AxisDirection car_header_direction = X_POSITIVE;

    // 钢卷的信息: 用于相关的参数的校验
    float coil_max_width = 1600;      // 钢卷的最大宽度
    float coil_max_diameter = 2500;     // 钢卷的最大半径

    // 是否自动裁剪: 如果为true则自动推测裁剪区域，如果为false则采用用户设置的区域
    bool cloud_auto_crop = true;
    // 区域位置：用于点云的裁剪，超出范围的会直接裁剪
    float cloud_crop_x_min = -10 * 1000;
    float cloud_crop_x_max = 10 * 1000;
    float cloud_crop_y_min = -10 * 1000;
    float cloud_crop_y_max = 10 * 1000;
    float cloud_crop_z_min = -2 * 1000;
    float cloud_crop_z_max = 0;

    // 单车厢信息: 汽车情况下表示汽车的长宽高，火车情况下表示车厢的长宽高
    float car_length = 15 * 1000;
    float car_width = 3 * 1000;
    float car_height = 2.5 * 1000;

    // 车厢分类时裁剪的头尾范围
    float head_tail_cut_range = 300;
    float highsided_height_min = 750;

    // 根据车厢范围裁剪车厢内点云的偏移量
    float crop_car_body_offset_y = 300;
    float crop_car_body_offset_x = 300;

    // 高栏车围栏内缩偏移量
    float fence_shrink_distance = 30;
    // 高栏车底部平面裁剪Z范围
    float highsided_bottom_z_range = 400;

    // 钢卷的裁剪偏移量
    float crop_coil_offset_x = 0;
    float crop_coil_offset_y = 100;       // 裁剪钢卷的时候Y方向的偏移量
    float crop_coil_offset_z_min = 500;
    float crop_coil_offset_z_max = 2500;

    // 鞍座的裁剪偏移量
    float crop_saddle_offset_x = 0;
    float crop_saddle_offset_y = 300;       // 裁剪鞍座的时候Y方向的偏移量
    float crop_saddle_offset_z_min = 150;
    float crop_saddle_offset_z_max = 400;

    // 鞍座尺寸阈值
    float saddle_length_min = 1500;
    float saddle_length_width_ratio_min = 2.0;
    float saddle_length_width_ratio_max = 5.0;
    float saddle_height_min = 80;
    float saddle_height_max = 155;
    float highsided_saddle_cluster_tolerance = 100;
    float saddle_like_rect_min = 95;
    float part_saddle_like_rect_min = 80;
    float part_saddle_width_min_gap = 50;
    float part_saddle_y_min_gap = 50;
    float part_saddle_max_gap = 150;

    // 钢卷判断的相关参数
    float coil_min_width = 900;
    float coil_min_diameter = 1500;
    float coil_side_crop_length = 60;    // 钢卷的侧面裁剪长度, 主要用于剔除钢卷的侧面点云，避免影响拟合
    float coil_axis_crop_threshold = 50; // 钢卷的轴向裁剪阈值，主要判断最低点和边界的距离，如果小于这个则认为不需要裁剪
    float coil_axis_crop_offset = 10;    // 钢卷的轴向裁剪偏移量，主要用于多裁剪一点，使得凹点被裁剪掉
    float is_coil_min_length_threshold = 100;   // 判断钢卷点云X方向长度的阈值，注意判断是最高点和点云边缘的距离 

    // 车厢点云降噪SOR参数
    int carriage_sor_mean_k = 20;
    float carriage_sor_stddev_mul_thresh = 2;

    // 整体鞍座点云降噪SOR参数
    int saddles_sor_mean_k = 50;
    float saddles_sor_stddev_mul_thresh = 2;
    // 单个鞍座点云降噪SOR参数
    int saddle_sor_mean_k = 10;  // 要小于saddle_cluster_min_size
    float saddle_sor_stddev_mul_thresh = 2;

    // 整体钢卷点云降噪SOR参数
    int coils_sor_mean_k = 20;
    float coils_sor_stddev_mul_thresh = 2;
    // 单个钢卷点云降噪SOR参数
    int coil_sor_mean_k = 20;
    float coil_sor_stddev_mul_thresh = 2;

    // 车厢聚类的参数: 用于从多个车厢中提取出单个车厢点云聚类
    float carriage_cluster_tolerance = 200;  // 车厢聚类容差
    int carriage_cluster_min_size = 1000;   // 车厢聚类最小的点数

    // 鞍座特征提取聚类参数
    float saddle_cluster_tolerance = 200;  // 鞍座聚类容差
    int saddle_cluster_min_size = 15;   // 鞍座聚类最小的点数

    // 钢卷特征提取聚类参数
    float coil_cluster_tolerance = 200;  // 钢卷聚类容差
    int coil_cluster_min_size = 100;   // 钢卷聚类最小的点数

    // 火车相关
    float train_inner_crop_offset = 300; // 火车内部裁剪偏移量，主要用于裁剪掉火车内部的点云
    float train_top_border_crop_min_offset_z = 200; // 裁剪获取顶部边框的底部Z偏移量
    float train_top_border_crop_max_offset_z = 1000; // 裁剪获取顶部边框的顶部Z偏移量

    // 用于校验的参数
    float min_carriage_interval = 10 * 1000;    // 车厢之间的最小间隔
    float min_saddle_interval = 1 * 1000;     // 鞍座之间的最小间隔
    float min_saddle_coil_interval_x = 0.45 * 1000;  // 鞍座X地址和钢卷边缘（注意不是钢卷的中心X）的最小间隔
    float min_saddle_coil_interval_y = 0.2 * 3000;//1000;  // 鞍座Y地址和钢卷边缘（注意不是钢卷的中心Y）的最小间隔 
    float min_coil_carriage_interval_x = 0.2 * 1000;  // 钢卷边缘X地址和车厢边缘的最小间隔
    float min_coil_carriage_interval_y = 0.01 * 1000;  // 钢卷边缘Y地址和车厢边缘的最小间隔
    float min_saddle_carriage_interval_x = 0.6 * 1000;  // 鞍座边缘X地址和车厢边缘的最小间隔
    float min_saddle_carriage_interval_y = 0.6 * 1000;  // 鞍座边缘Y地址和车厢边缘的最小间隔

    [[nodiscard]] Axis carAxis(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in) const; 

    // 获取扫描区域的大小
    [[nodiscard]] std::tuple<float, float, float> areaSize() const {
        return {abs(cloud_crop_x_max - cloud_crop_x_min), abs(cloud_crop_y_max - cloud_crop_y_min), abs(cloud_crop_z_max - cloud_crop_z_min)};
    }

    /**
     * @brief 从json文件加载配置
     * @param path 
     */
    void loadConfigFromJson(const std::string &path);

    /**
     * @brief 将配置保存到json文件
     * @param path 
     */
    void saveConfigToJson(const std::string &path) const;
};

#endif //SCAN_CONFIG_H
