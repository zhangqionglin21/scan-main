#ifndef SCAN_SCAN_H
#define SCAN_SCAN_H

#include <set>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include "config.h"
#include "chassis.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/io.h>
#include <pcl/common/pca.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/segmentation/sac_segmentation.h>

// 平板货车
#define FLATBED_TRUCK 0
// 高栏式货车
#define HIGHSIDED_TRUCK 1

/**
 * 扫描类的公共实现
 */
class Scan {
public:
    ScanConfig config;

protected:
    const std::string default_config_path = "scan_config.json";

    std::vector<Chassis> chassis = {};
public:
    /**
     * 默认构造函数: 不加载配置使用默认配置
     */
    Scan() {
        boost::log::register_simple_formatter_factory<boost::log::trivial::severity_level, char>("Severity");

        boost::log::add_file_log(
                boost::log::keywords::file_name = "logs/%Y%m%d_%H%M%S.log",
                boost::log::keywords::rotation_size = 10 * 1024 * 1024,
                boost::log::keywords::auto_flush = true
        );
        boost::log::add_common_attributes();
    };

    /**
     * 通过配置构造
     * @param default_config 
     */
    [[maybe_unused]] explicit Scan(ScanConfig default_config);

    /**
     * 通过配置构造
     * @param config 
     */
    explicit Scan(const std::string &path);

    /**
     * 获取默认配置, 主要用于子类的默认配置
     */
    virtual ScanConfig getDefaultConfig();

    /**
     * 加载扫描配置
     * @param path 
     */
    bool loadConfig(const std::string &path);

    /**
     * 检测
     * @return 
     */
    virtual std::vector<DetectResult> detectFromFile(const std::string &path);

    /**
     * 检测
     * @return 
     */
    std::vector<DetectResult>
    detect(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ> *cloud_out);

    /**
     * @brief 对整体点云进行预处理，主要进行范围的裁剪，剔除地面等 
     * @param cloud_in 原始点云
     * @param config 配置信息
     * @return 预处理后的点云 
     */
    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr cleanPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in);

    /**
     * @brief 获取车厢的点云
     * @param cloud_in 输入的点云
     * @return 货车的点云
     */
    virtual std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> getCarriageClouds(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in);

    /**
     * @brief 获取车厢类型及车厢角度
     * @param cloud_in 输入的点云
     * @return 车厢类型: 0-平板货车，1-高栏式货车
     */
    virtual std::pair<int, float> getCarriagesType(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, int switch_flag);

    /**
     * @brief 计算点云平均高度
     * @param cloud 输入点云
     * @return 点云平均高度
     */
    float calculateAverageHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    /**
     * @brief 从整体的点云中提取出车厢信息，包含车厢部分的点云（一般不包含围栏），车厢的边界信息
     * @param cloud_in 输入的点云
     * @return 返回车厢的信息，车厢信息默认按照x轴从小到大排序 
     */
    virtual std::vector<Carriage> getCarriages(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in) = 0;

    virtual Carriage getCarriage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& car_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& carriage_cloud) = 0;
    
    virtual bool getCarriagesSingle(const pcl::PointCloud<pcl::PointXYZ>::Ptr& train_cloud, float x1, float y1, float x2, float y2, Carriage& output) = 0;
    /**
     * @brief 对车厢结果排序，按照x轴从小到大排序
     * @param carriages 车厢信息
     * @return 排序后的车厢信息 
     */
    static std::vector<Carriage> sortCarriages(const std::vector<Carriage> &carriages);

    /**
     * @brief 从整体或者单个的点云中提取鞍座和钢卷的点云
     * @param carriage 车厢范围以及点云信息
     * @return 
     */
    virtual std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr>
    getSaddleAndCoilCloud(const Carriage &carriage);

    /**
     * 提取鞍座点云
     * @param cloud_in 输入的点云，该点云应该是做过裁剪，而且不包含的钢卷的点云
     * @param saddle_cloud 提取的鞍座的点云（鞍座整体的点云） 
     * @param coefficients 车厢平面的方程 
     * @return 
     */
    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr getSaddleCloud(const Carriage &carriage);

    /**
     * 提取高栏车鞍座点云
     * @param cloud_in 输入的点云，该点云应该是做过裁剪，而且不包含的钢卷的点云
     * @param saddle_cloud 提取的鞍座的点云（鞍座整体的点云）
     * @param coefficients 车厢平面的方程
     * @return
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr Scan::getHighsidedSaddleCloud(const Carriage& carriage);

    /**
     * 获取钢卷的点云
     * @param cloud_in 输入的点云
     * @param coefficients 车厢平面的方程
     * @param car_min_min 顶点1
     * @param car_min_max 顶点2
     * @param car_max_min 顶点3
     * @param car_max_max 顶点4
     */
    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr getCoilCloud(const Carriage &carriage);

    /**
     * 识别钢卷
     * @param coils_cloud 包含所有钢卷的点云
     * @param config 配置信息
     * @return 识别的点云信息 
     */
    virtual std::vector<DetectResult>
    detectCoils(const pcl::PointCloud<pcl::PointXYZ>::Ptr &coils_cloud);

    /**
     * @brief 从钢卷点云中拆分出单个的钢卷点云
     */
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
    getCoilClouds(const pcl::PointCloud<pcl::PointXYZ>::Ptr &coils_cloud) const;

    /**
     * 识别单个钢卷
     * @param coil_cloud 单个钢卷的点云
     * @param config 配置信息 
     * @return 
     */
    virtual std::shared_ptr<DetectResult>
    detectCoil(const pcl::PointCloud<pcl::PointXYZ>::Ptr &coil_cloud);

    /**
     * 对单个钢卷点云进行降噪
     * @param coil_cloud 单个钢卷的点云
     * @param scan_config 配置信息 
     * @return 
     */
    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr
    deNoiseCoilCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &coil_cloud);

    /**
     * 判断点云是否是钢卷点云
     * @param coil_cloud 
     * @param config 
     * @return 
     */
    virtual bool isCoilCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &coil_cloud);

    /**
     * 对单个钢卷点云进行拟合
     * @param coil_cloud 钢卷的点云
     * @param config 
     * @return 
     */
    static DetectResult fittingCoil(const pcl::PointCloud<pcl::PointXYZ>::Ptr &coil_cloud, const ScanConfig &config);


    /**
     * 识别鞍座
     * @param saddles_cloud 包含所有鞍座的点云
     * @param config 配置信息
     * @param coils_result 钢卷的识别结果, 可用于辅助识别鞍座，比如可剔除在钢卷范围内的鞍座
     * @return 
     */
    virtual std::vector<DetectResult>
    detectSaddles(const pcl::PointCloud<pcl::PointXYZ>::Ptr &saddles_cloud,
                  const std::vector<DetectResult> &coils_result, const Carriage &carriage);

    /**
     * 获取最佳匹配的模式，从所有的点中搜索出最佳多种组合
     * @param points 待查找的点集合 
     * @param patterns 参考模式集合 
     * @return 最佳的模式的索引和匹配的点集合 
     */
    static std::vector<std::pair<int, std::vector<std::pair<Point, Point>>>>
    getBestMatchPatterns(const std::vector<pcl::PointXYZ> &points, std::vector<std::vector<Point>> &patterns);

    /**
     * 获取鞍座的特征点：从鞍座的突出的点云提取一个最具有代表性的点(往往是高点)
     * @param saddle_cloud 鞍座突出点的点云，一个鞍座可能有四个突出点，这里只需要其中一个 
     * @param config 配置信息 
     * @return 返回的是鞍座的特征点 
     */
    virtual pcl::PointXYZ
    getSaddleFeaturePoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr &saddle_cloud);


    /**
     * 合并识别结果，库位和钢卷的识别需要相互印证，库位可能多识别出来，比如相同位置已经识别出了钢卷，那么这个时候下面就不应该再识别出库位
     * @param coils_cloud 钢卷的点云
     * @param coils_result 钢卷的识别结果 
     * @param saddles_cloud 鞍座的点云 
     * @param saddles_result 鞍座的识别结果 
     * @param car_border 车厢的边界
     * @param car_index 车厢的索引
     * @return 合并后的结果，并且添加上的车厢的信息结果
     */
    virtual std::vector<DetectResult>
    mergeDetectResult(const pcl::PointCloud<pcl::PointXYZ>::Ptr &coils_cloud,
                      const std::vector<DetectResult> &coils_result,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr &saddles_cloud,
                      const std::vector<DetectResult> &saddles_result,
                      const MinMaxPoint &car_border,
                      int car_index);
    
    /**
     * @brief 格式化检测结果，正常不做任何处理，主要是根据特殊情况进行处理, 比如汽车的高低差的车厢
     * @param results 检测的结果
     * @return 格式化后的结果
     */
    virtual std::vector<DetectResult> formatDetectResult(const std::vector<DetectResult> &results);

    /**
     * @brief 根据钢卷的结果剔除在钢卷范围内的库位，因为就算有钢卷也可能识别到鞍座，但是如果有卷则不应该返回鞍座，并且钢卷的识别相对准确，所以优先信任钢卷的识别结果
     * @param coil_results 钢卷识别结果
     * @param saddle_results 鞍座识别结果 
     * @param x_offset 判断范围额外要扩大的范围, 如果为负数则缩小范围 
     * @param y_offset 判断的范围额外要扩大的范围, 如果为负数则缩小范围 
     * @return 
     */
    static std::vector<DetectResult> filterDetectSaddleWithCoil(const std::vector<DetectResult> &coil_results,
                                                                const std::vector<DetectResult> &saddle_results,
                                                                float x_offset = 300, float y_offset = 300);

    /**
     * @brief 根据车厢范围以及偏移值，过滤鞍座识别结果
     * @param results
     * @param car_border
     * @param x_offset
     * @param y_offset
     */
    static std::vector<DetectResult> filterDetectSaddleWithCarBorder(const std::vector<DetectResult> &results,
                                                                     const MinMaxPoint &car_border,
                                                                     float x_offset = 200, float y_offset = 200);

    /**
     * @brief 根据车厢范围以及偏移值，过滤钢卷识别结果
     * @param results 
     * @param car_border 
     * @param x_offset 
     * @param y_offset 
     * @return 
     */
    static std::vector<DetectResult> filterDetectCoilWithCarBorder(const std::vector<DetectResult> &results,
                                                                   const MinMaxPoint &car_border,
                                                                   float x_offset = 200, float y_offset = 200);

    /**
     * @brief 根据钢卷属性，过滤钢卷识别结果
     * @param results 钢卷识别结果 
     * @param max_coil_with 最大宽度
     * @param max_coil_diameter 最大直径
     * @return 
     */
    static std::vector<DetectResult> filterDetectCoilWithCoilParam(const std::vector<DetectResult> &results,
                                                                   float max_coil_with = 2000,
                                                                   float max_coil_diameter = 2000);


    // 调试等使用的辅助函数

    /**
     * 判断路径是文件还是目录，如果是文件则获取父目录，并且判断是否存在如果不存在则创建
     * @param path 文件路径
     */
    static void createDirectory(const std::string &path);

    /**
     * 将检测结果转换为点
     * @param result 检测结果
     * @return 点的集合 
     */
    static std::vector<Point> detectResultToPoint(const std::vector<DetectResult> &result);

    /**
     * 创建点云
     * @param points 根据点的集合创建点云
     * @return 点云对象 
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr createCloud(const std::vector<Point> &points);

    /**
     * 保存点到pcd文件
     * @param points 
     * @param path 
     */
    static void savePointToPcd(const std::vector<Point> &points, const std::string &path);

    /**
     * 保存点云到pcd文件
     * @param cloud 
     * @param path 
     * @param is_binary 
     */
    static void
    saveCloudToPcd(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &path, bool is_binary = false);

    /**
     * 当属于调试模式的时候，保存点云到文件
     * @param cloud 
     * @param name 
     */
    void saveCloudWhenDebug(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &name) const;

    /**
     * @brief 将检测结果转换为点云
     * @param results 
     * @return 
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr switchResultToCloud(std::vector<DetectResult> &results);

    /**
     * @brief 将点转换为点云
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr switchResultToCloud(std::vector<pcl::PointXYZ> &in_points);

    /**
     * @brief 自动缩放点云，主要是输入的点云先前是存在单位不同的情况，部分是m，而最新的是mm，所以尝试根据期望的大小判断是否是m，如果是m则转换为mm
     * @param cloud_in 输入点云 
     * @param excepted_x_size 期望的X方向长度
     * @param excepted_y_size 期望的Y方向长度
     * @param excepted_z_size 期望的Z方向长度
     * @return 缩放后的点云
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr
    autoScaleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float excepted_x_size, float excepted_y_size,
                   float excepted_z_size);

    /**
     * @brief 检测结果的检查，主要是检查是否有重复的点，或者点的坐标是否合理 
     * @param results 检测的结果 
     * @return 转换后的结果 
     */
    void checkDetectResults(const std::vector<DetectResult> &results) const;

    /**
     * @brief 根据类型获取检测结果，并且按照X从小到大排序
     * @param results 检测的结果 
     * @param types 检测的类型 
     * @return 排序后的结果 
     */
    [[nodiscard]] static std::vector<DetectResult>
    getSortedDetectResultsByTypes(const std::vector<DetectResult> &results, const std::vector<DetectType> &types);

    /**
     * @brief 根据配置信息中的配置尝试交换点云的xy坐标，因为目前算法中全部是假定X轴是车厢的长度方向，Y轴是宽度方向，但是有些点云可能是反的，所以需要尝试交换
     * @param cloud_in 输入点云 
     * @return 交换后的点云 
     */
    [[nodiscard]] pcl::PointCloud<pcl::PointXYZ>::Ptr
    trySwitchCloudXy(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, int& switch_flag) const;

    /**
     * @brief 尝试交换检测结果的xy坐标, 和trySwitchCloudXy中的逻辑需要保持一致
     * @param results 检测的结果 
     * @return 交换后的检测结果 
     */
    [[nodiscard]] std::vector<DetectResult>
    trySwitchResultXy(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                      const std::vector<DetectResult> &results, int switch_flag) const;

    /**
     * @brief 交换点云的xy坐标
     * @param cloud_in 输入点云 
     * @return 交换后的点云 
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr
    switchXy(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in);

    /**
     * @brief 交换检测结果的xy坐标
     * @param results 检测的结果 
     * @return 交换后的检测结果 
     */
    static std::vector<DetectResult>
    switchXy(const std::vector<DetectResult> &results);

    /**
     * @brief 查找从X轴看，当Z最小的点对应的X
     * @param cloud_in 
     * @return 
     */
    static pcl::PointXYZ findFirstMinZPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in);

    /**
     * @brief 格式化输出点云
     * @param cloud: 输入点云
     * @param cloud_out: 输出用的点云
     * @param results: 识别的结果，用于辅助裁剪
     */
    virtual void
    formatOutputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ> *cloud_out,
                      const std::vector<DetectResult> &results);

    Eigen::Vector3f normalize(const Eigen::Vector3f& v);

    std::vector<Eigen::Vector3f> shrinkQuadrilateral(const std::vector<Eigen::Vector3f>& original_points, float shrink_distance);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cropPointCloudWithQuadrilateral(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        const std::vector<Eigen::Vector3f>& boundary_points);

    pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudDifference(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_a,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_b,
        float tolerance = 0.001f);

    std::vector<DetectResult> detectHighsidedSaddles(pcl::PointCloud<pcl::PointXYZ>::Ptr& saddles_cloud, const std::vector<DetectResult>& coils);

    float getGlobalAverageHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    float computeMinBoundingBoxAnglePCA(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);

    int calculateRotatedRectCoverage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    static float formatAngle(float angle, int offset);

    float computeUnwindingAngleFromSaddle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};


#endif //SCAN_SCAN_H
