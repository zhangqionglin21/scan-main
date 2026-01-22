#ifndef SCAN_SCAN_EXPORT_H
#define SCAN_SCAN_EXPORT_H

// 定义版本
#define SCAN_VERSION "0.10.0"

/**
 * @brief 火车鞍座识别，单车厢？
 * @param cloud_in 输入点云
 * @param cloud_out 输出点云
 * @param params 输出的车厢顶点
 * @param param_count 车厢顶点个数
 */
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include "types.h"

extern "C" __declspec(dllexport) void
getTrainVertex(pcl::PointCloud<pcl::PointXYZ> *cloud_in, pcl::PointCloud<pcl::PointXYZ> *cloud_out,
               DetectResult **params, int &param_count);


/**
 * @brief 火车鞍座识别,多车厢
 * @param cloud_in 输入点云
 * @param cloud_out 输出点云
 * @param params 输出的车厢顶点
 * @param param_count 车厢顶点个数
 */
extern "C" __declspec(dllexport) void
getTrainSaddle(pcl::PointCloud<pcl::PointXYZ> *cloud_in, pcl::PointCloud<pcl::PointXYZ> *cloud_out,
               DetectResult **params, int &param_count);

// filename为点云文件名
// params为输出的ModelParam
// point_n为ModelParam个数
extern "C" __declspec(dllexport) void
getTrainVertexFromCloud(const char *filename, DetectResult **params, int &param_count);

/**
 * @brief 汽车扫描导出
 */
extern "C" __declspec(dllexport) void
getCarVertex(pcl::PointCloud<pcl::PointXYZ> *cloud_in, pcl::PointCloud<pcl::PointXYZ> *cloud_out,
             DetectResult **params, int &param_count);


extern "C" __declspec(dllexport) void
getCarVertexTest(pcl::PointCloud<pcl::PointXYZ> *cloud_in, pcl::PointCloud<pcl::PointXYZ> *cloud_out,
                 DetectResult **params, int &param_count);

/**
 * @brief 汽车AGV扫描导出
 */
extern "C" __declspec(dllexport) void
getAgvVertex(pcl::PointCloud<pcl::PointXYZ> *cloud_in, pcl::PointCloud<pcl::PointXYZ> *cloud_out,
             DetectResult **params, int &param_count);


/**
 * 裁剪点云
 */
extern "C" __declspec(dllexport) void
cropCloudXyz(pcl::PointCloud<pcl::PointXYZ> *cloud_in, pcl::PointCloud<pcl::PointXYZ> *cloud_out, float min_x,
             float min_y, float min_z, float max_x, float max_y, float max_z);

#endif //SCAN_SCAN_EXPORT_H
