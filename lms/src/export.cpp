#include "export.h"
#include "LMS3DScanPlatformDll.h"
#include "ScanPlatformData.h"

// 3D扫描云台设备类型(一定要正确选择)
enum EPlatFormType {
    UnKnown_Type = 1,
    ECO_LMS_511 = 2,    //   轻量型版LMS 511
    ECO_LMS_111 = 3,    //   轻量型版LMS 111
    ECO_NANOSCAN = 4,    //   轻量型版Nanoscan
    ECO_LRS4000 = 5,    //   轻量型版LRS4000
    Standard_LMS_511 = 5110,    //   标准版LMS 511，暂不支持
    ARM_LMS_511 = 5111,            //   悬臂版LMS 511，暂不支持
    ARM_LMS_111 = 111,            //   悬臂版LMS 111，暂不支持
    LD_LRS_36x0 = 3600,            //	 LD-LRS 3600云台，暂不支持
    LD_LRS_36x1 = 3601,            //   LD-LRS 36x1云台，暂不支持
};

// 全局平台实例
static LMS3DScanPlatForm *platForm = new LMS3DScanPlatForm();
static EPlatFormType deviceType = EPlatFormType::ECO_LMS_511;


bool Init(char *lmsIPAddr, int lmsPort, char *controlSysIPAddr, int controlSysPort, int &errorCode) {
//    // 如果示例存在，则尝试释放
//    if (platForm != nullptr) {
//        platForm->Destory();
//        delete platForm;
//    }
//
//    // 重新构造
//    platForm = new LMS3DScanPlatForm();

    // 指定初始化
    auto platformErr = CommonLms3DScanPlatformError::NO_PLATFORMERROR;
    auto result = platForm->Init(
            platformErr,
            deviceType,
            lmsIPAddr,
            lmsPort,
            controlSysIPAddr,
            controlSysPort
    );
    if (!result) {
        errorCode = platformErr;
    }

    return result;
}

/// <summary>
/// 设置扫描参数
/// </summary>
/// <param name="startAngle">扫描起始角，度。</param>
/// <param name="stopAngle">扫描结束角，度。</param>
/// <param name="vel">扫描角速度， 度/秒。</param>
/// <param name="errorCode">错误代码</param>
/// <returns>布尔值 成功返回:true;失败返回:false;</returns>
bool SetParameter(int startAngle, int stopAngle, int vel, int &errorCode) {
    auto platformErr = CommonLms3DScanPlatformError::NO_PLATFORMERROR;
    auto result = platForm->SetScanParameter(startAngle, stopAngle, vel, platformErr);
    if (!result) {
        errorCode = platformErr;
    }
    return result;

}

/// <summary>
/// 获取扫描参数
/// </summary>
/// <param name="startAngle">扫描起始角，度。</param>
/// <param name="stopAngle">扫描结束角，度。</param>
/// <param name="vel">扫描角速度， 度/秒。</param>
void GetParameter(int &startAngle, int &stopAngle, int &vel) {
    platForm->GetScanParameter(startAngle, stopAngle, vel);
}

/// <summary>
/// 触发扫描
/// 函数作用：
/// 1，使平台开始旋转进行扫描。只有调用该方法进行扫描以后，系统才能产生 3D 点云数据。
///	2，系统最多会保存最近 5 次扫描所得的点云数据，若用户成功调用了 5 次 Start3DScanOnce，
///	但是一直不通过 Get3DPointCloud 或者 GetNewest3DPointCloud 函数获取点云，那么第 6 次调用
/// Start3DScanOnce 不通将会覆盖第一次的点云数据，第 7 次调用 Start3DScanOnce 将会覆盖
/// 第 2 次的点云数据，以此类推
/// </summary>
/// <param name="errorCode">错误代码</param>
/// <returns>布尔值 成功返回:true;失败返回:false;</returns>
bool StartOnce(int &errorCode) {
    auto platformErr = CommonLms3DScanPlatformError::NO_PLATFORMERROR;
    auto result = platForm->Start3DScanOnce(platformErr);
    if (!result) {
        errorCode = platformErr;
    }
    return result;
}

/// <summary>
/// 获取3D点云数据
/// </summary>
/// <param name="errorCode">错误代码</param>
/// <param name="length">点云长度</param>
/// <param name="width">点云宽度</param>
/// <returns>点云数据</returns>
Point3D *__stdcall GetPoint(int &errorCode, int &length, int &width) 
{
    Double3DPointVec cloudData;
    errorCode = platForm->GetNewest3DPointCloud(cloudData);
    if (errorCode == 1) {
        length = static_cast<int>(cloudData.size());
        width = static_cast<int>(cloudData[0].size());
        auto *pt = new Point3D[length * width];
        for (int i = 0; i < length; i++) {
            for (int j = 0; j < width; j++) {
                auto point = cloudData[i][j];
                *(pt + (i * width) + j) = point;
            }
        }
        return pt;
    } else {
        length = 0;
        width = 0;
    }
    return nullptr;
}

/// <summary>
/// 设置定点扫描角度
/// </summary>
/// <param name="iStopAngle">定点扫描角度</param>
/// <param name="iVelocity">速度</param>
/// <param name="rrrCode">错误代码</param>
/// <returns>布尔值 成功返回:true;失败返回:false;</returns>
bool StartMoveToAnglePosition(int iStopAngle, int iVelocity, int &errorCode) 
{
    // 不支持: 这个是为了兼容后面的才添加的函数，本身是不支持的
    return false;
}


/// <summary>
/// 获取定点扫描单帧轮廓数据
/// </summary>
/// <param name="ErrCode">错误代码</param>
/// <param name="length">点云长度</param>
/// <returns>单帧点云数据</returns>
Point2D *__stdcall GetLMSSingleProfile(int &ErrCode, int &length) 
{
    // 不支持: 这个是为了兼容后面的才添加的函数，本身是不支持的
    return nullptr;
}

void Destory()
{
    platForm->Destory();
}
