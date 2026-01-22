#pragma once
#ifdef  LMS3DSCANDLL
#define LMS3DSCANAPI _declspec(dllexport)
#else
#define LMS3DSCANAPI  _declspec(dllimport)
#endif
#include <winsock2.h>
#include <iostream>
#include <cstring>
#include <queue>
#include "ScanPlatformData.h"
class	ControlSystemTcpComm;
class	LmsSensor;
//LMS3D_LIB-NOTFOUND.lib
enum CommonLms3DScanPlatformError
{
    NO_PLATFORMERROR = 0,
    LMS_CONNECT_FAIL = 1,                     //连接LMS传感器失败
    LMS_LOGIN_FAIL = 2,                       //登陆LMS传感器失败
    LMS_START_MEASURE_FAIL = 3,               //使LMS开始测量失败
    LMS_CREATE_ScanTHREAD_FAIL = 4,           //创建LMS扫描线程失败
    CONTROL_SYS_SOCKET_CREATE_FAIL = 5,        //创建控制系统套接字失败
    CONTROL_SYS_SOCKET_CONNECT_FAIL = 6,	   //与云台控制系统连接失败
    CONTROL_SYS_ENABLE_MOTOR_FAIL = 7,		   //使能电机指令发送失败
    CONTROL_SYS_CREATE_HeartBtTHREAD_FAIL = 8, //创建与云台控制系统的心跳交互线程失败
    CONTROL_SYS_CREATE_RecvMsgTHREAD_FAIL = 9,//创建接收来自于云台控制系统的消息的线程失败
    CONTROL_SYS_HeartBtUnnormal = 10,		   //与云台控制板之间的心跳异常
    CONTROL_SYS_SET_PARAM_ERR = 11,			   //设置的扫描参数错误
    CONTROL_SYS_SEND_COMMAND_FAIL = 12,        //发送消息至云台控制器失败
    PLATEFORM_IS_MOVING_NEED_WAITTING = 13,    //平台已经在执行动作，请稍后。
    PLATEFORM_NOT_INIT = 14,				   //平台未初始化
    PLATEFORM_RES_INIT_FAIL = 15,              //平台底层初始化分配资源失败
    PLATFORM_CALCULATING_DATA = 16,            //平台正在执行点云转换计算，请稍后
    PLATFORM_MOTOR_NOENABLE = 17,			        //云台电机还未使能
    PLATFORM_InitialStage_ToZeroPoint = 18,         //云台还在初始化找零点中
    PLATFORM_InitialStage_ToStartAngle = 19,        //云台初始化，去StartAngle过程中
    PLATFORM_InitialStage_WaitToSetScanAngle = 20,  //等待设置扫描起始角/结束角/角速度
    PLATFORM_MOTOR_POS_ERR = 21,					//控制器位置报错
};

enum EModelFeedbackStatus
{
    eIdleStatus = 0,//初始化，没状态
    eSingleMoving_Clock_S1 = 1,//单次运行，正转（从小角度->大角度）
    eRepeatMoving_Clock_SB = 2,//往复运动，正转中
    eSingleMoving_ClockWise_S2 = 3,//单次运行，反转中
    eRepeatMoving_ClockWise_SC = 4,//往复运动，反转中
    eStopAtStartPosition_S3 = 5,   //停在起始位置
    eStopAtStopPosition_S4 = 6,    //停在结束位置
    eNOEnable_S5 = 7,              //未Enable
    eInitialStage_ToZeroPoint_S6 = 8,//初始化，找零中
    eInitialStage_WaitToSetScanAngle_S7 = 9,//初始化，等待设置起始/结束角度 
    eInitialStage_ToStartAngle_S8 = 10,//初始化，去StarAngle过程中
    eMotorPosErr_S9 = 11,//MotorPosError，pos_err_stop=1
    eNOHB_ii = 12//10s内未接到上位机“HB”心跳
};


class LMS3DSCANAPI LMS3DScanPlatForm
{
public:
LMS3DScanPlatForm();
virtual ~LMS3DScanPlatForm(void);

/*
**   分配资源,开启工作线程
*@ err：        错误枚举值
*@deviceIndex:  设备索引号，方便区分不同的云台
*@ lmsIPAddr:    传感器IP地址
*@ lmsPort：     传感器服务端口
*@ controlSysIPAddr:  云台控制器IP地址
*@ controlSysPort:    云台控制器端口
*@ dUserAngleOffsetToCordXAxis：用户用来控制云台绝对坐标系Z轴方向的控制量，一般填0.0即可（单位:度）
*@ iDistanceOffsetToRotateCenter: 传感器自身测量中心距电机旋转轴的距离  （单位:dm）
*@ angOffsetToHorizontal: 当旋转平台处于水平面时，编码器的角度     (单位:度)
*/
bool    Init(CommonLms3DScanPlatformError& err,
             unsigned short          deviceIndex,
             const std::string&		lmsIPAddr,
             unsigned short			lmsPort,
             const std::string&		controlSysIPAddr,
             unsigned short			controlSysPort = 5000,
             double					dUserAngleOffsetToCordXAxis = 0.0,
             double					iDistanceOffsetToRotateCenter = 0.66,
             double					angOffsetToHorizontal = 0.0);

//销毁资源与子线程
void    Destory();

/*
**   获取LMS传感器产品序列号
*@ 返回值：0表示未能成功获取(诸如云台未初始化、传感器连接异常等等均会导致不能成功获取该序列号)
*/
unsigned int GetLmsSerialNo();

/*
**   获取云台传感器内部温度
*@ 返回值：温度值，0表示未能成功获取
*/
unsigned int GetTemperature();

/*
**   获取云台状态
*@ 返回值：0表示未能成功获取
*/
unsigned int GetStatus(EModelFeedbackStatus& status);


//Note: 经测试，在VS2015、VS2019版本中使用方式一可行。
//（方式一 ！！！）获取点云数据，以下两个函数适合用在MD及MDd编译选项下   
// 返回值: 1:正常获取到点云数据;  0:当前无可用点云数据;  -1: 模块内部工作停止，需要重新调用Init
int    Get3DPointCloud(Double3DPointVec& cloudData);
int    GetNewest3DPointCloud(Double3DPointVec& cloudData);

//Note: 经测试，在VS2013版本中使用方式二可行，方式一不可行。若为VS2013及以下版本，推荐使用方式二。
//（方式二 ！！！）获取点云数据，以下两个函数适合用在MT及MTd编译选项下（确保vector的内存分配与释放都在dll中进行）
//函数在使用时需要传一个NULL指针进去
//eg:    Double3DPointVec* vec = NULL;
//       Get3DPointCloud(&vec);
//使用完成之后需要调用 DestroyVectorMemory(&vec);
//返回值: 1 : 正常获取到点云数据;  0:当前无可用点云数据;  -1: 模块内部工作停止，需要重新调用Init
//       -2 : 请传入NULL(*ppCloudData != NULL)
int 	Get3DPointCloud(Double3DPointVec** ppCloudData);
int     GetNewest3DPointCloud(Double3DPointVec** ppCloudData);
void	DestroyVectorMemory(Double3DPointVec** ppVector);

/*
**   设置扫描参数(起始角/结束角/角速度)
*@ startAngle： 扫描起始角(度)  >=60
*@ stopAngle:   扫描结束角(度)  <=230
*@ vel:		    扫描速度，度/秒  大于等于1且小于360
*@ err：        错误枚举值
*@ 返回值: true成功，false失败（失败时具体原因参见err）
*/
bool SetScanParameter(int startAngle,int stopAngle,int vel,CommonLms3DScanPlatformError& err);
void GetScanParameter(int& startAngle, int& stopAngle, int& vel);

//开启单次3D扫描
bool Start3DScanOnce(CommonLms3DScanPlatformError& err);

//获取当前设备的索引号，便于区分
unsigned short GetDeviceIndex();

//模块内部线程是否均在正常运行，如果不正常，重新Init
bool    IsThreadRunningOK();

protected:

private:
void	ClearResource();
void	Add3DCloudToQue(const Double3DPointVec& cloudData);

//LMS扫描线程的回调
friend void LMS3DScanThreadCallbk(void* para);
friend void ControlSystemHeartBtThreadCallbk(void* para);

//成员变量区
private:
LmsSensor*						m_pLmsSensor;
ControlSystemTcpComm*			m_pControlSysSocket;
bool							m_isInitOk;         //是否成功进行了初始化
bool                            m_isChildThreadRunning;
bool                            m_isCalculatePointOK;

std::queue<Double3DPointVec>    m_3dCloudQue;
HANDLE							m_h3dCloudQueMutex;
SLms3DPoint3DCloud				m_sLms3DPoint3DCloud;

void*                           m_laserScanThread;
unsigned    long                m_laserScanThreadID;
void*							m_heartBtThread;
unsigned    long                m_heartBtThreadID;

unsigned    int					m_lmsSerialNo;

int								m_startAngle;
int								m_stopAngle;
int                             m_scanVelocity;     //  (度/秒 ,velocity <= 360)
unsigned short					m_deviceIndex;
//以下三个成员变量属于系统的标定参数
double				m_iDistanceOffsetToRotateCenter;//传感器中心与旋转中心的安装偏移(单位:分米)  
double              m_user_dAngleOffsetToCordZAxi;   //用户用来控制云台绝对坐标系Z轴方向的控制量，一般填0.0即可（单位:度）
double              m_angOffsetToHorizontal;        //当旋转平台处于水平面时，编码器的角度(单位:度)
};

