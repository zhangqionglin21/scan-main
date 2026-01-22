#ifndef LMS_SCAN_210_EXPORT_H
#define LMS_SCAN_210_EXPORT_H

// 定义版本
#include "ScanPlatformData.h"

#define LMS_VERSION "2.3.0"

/**
 * @brief 火车鞍座识别，单车厢？
 * @param cloud_in 输入点云
 * @param cloud_out 输出点云
 * @param params 输出的车厢顶点
 * @param param_count 车厢顶点个数
 */
extern "C" __declspec(dllexport) bool Init(char *lmsIPAddr, int lmsPort, char *controlSysIPAddr, int controlSysPort, int &errorCode);

/// <summary>
/// 设置扫描参数
/// </summary>
/// <param name="startAngle">扫描起始角，度。</param>
/// <param name="stopAngle">扫描结束角，度。</param>
/// <param name="vel">扫描角速度， 度/秒。</param>
/// <param name="ErrCode">错误代码</param>
/// <returns>布尔值 成功返回:true;失败返回:false;</returns>
extern "C" __declspec(dllexport) bool SetParameter(int startAngle, int stopAngle, int vel, int &errorCode);

/// <summary>
/// 获取扫描参数
/// </summary>
/// <param name="startAngle">扫描起始角，度。</param>
/// <param name="stopAngle">扫描结束角，度。</param>
/// <param name="vel">扫描角速度， 度/秒。</param>
extern "C" __declspec(dllexport) void GetParameter(int &startAngle, int &stopAngle, int &vel);

/// <summary>
/// 触发扫描
/// 函数作用：
/// 1，使平台开始旋转进行扫描。只有调用该方法进行扫描以后，系统才能产生 3D 点云数据。
///	2，系统最多会保存最近 5 次扫描所得的点云数据，若用户成功调用了 5 次 Start3DScanOnce，
///	但是一直不通过 Get3DPointCloud 或者 GetNewest3DPointCloud 函数获取点云，那么第 6 次调用
/// Start3DScanOnce 不通将会覆盖第一次的点云数据，第 7 次调用 Start3DScanOnce 将会覆盖
/// 第 2 次的点云数据，以此类推
/// </summary>
/// <param name="ErrCode">错误代码</param>
/// <returns>布尔值 成功返回:true;失败返回:false;</returns>
extern "C" __declspec(dllexport) bool StartOnce(int &errorCode);


extern "C" __declspec(dllexport) void Destory();

/// <summary>
/// 获取3D点云数据
/// </summary>
/// <param name="ErrCode">错误代码</param>
/// <param name="length">点云长度</param>
/// <param name="width">点云宽度</param>
/// <returns>点云数据</returns>
extern "C" __declspec(dllexport) Point3D *__stdcall GetPoint(int &errorCode, int &length, int &width);

/// <summary>
/// 设置定点扫描角度
/// </summary>
/// <param name="iStopAngle">定点扫描角度</param>
/// <param name="iVelocity">速度</param>
/// <param name="ErrCode">错误代码</param>
/// <returns>布尔值 成功返回:true;失败返回:false;</returns>
extern "C" __declspec(dllexport) bool
StartMoveToAnglePosition(int iStopAngle, int iVelocity, int &errorCode);

/// <summary>
/// 获取定点扫描单帧轮廓数据
/// </summary>
/// <param name="ErrCode">错误代码</param>
/// <param name="length">点云长度</param>
/// <returns>单帧点云数据</returns>
extern "C" __declspec(dllexport) Point2D *__stdcall GetLMSSingleProfile(int &ErrCode, int &length);

#endif // LMS_SCAN_210_EXPORT_H
