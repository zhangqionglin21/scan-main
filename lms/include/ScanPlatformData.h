#pragma once
#include <string>
#include <vector>
struct Point2D
{
    Point2D() { X = 0; Z = 0; }
    float X;
    float Z;
};
struct Point3D
{
    Point3D() { X = 0; Y = 0; Z = 0; }
    float X;
    float Y;
    float Z;
};
typedef std::vector<Point3D> Single3DPointVec;
typedef std::vector<Single3DPointVec> Double3DPointVec;
//LMS激光传感器扫描得到的3D点云数据(还未转换为云台直角坐标系内的值)
struct SLms3DPoint3DCloud
{
    int								iSensorScanFreq;    // 传感器扫描频率
    std::vector<int>				vEncoder;           // 编码器读数
    Double3DPointVec	            vvPointsCloud;      // LMS传感器直角测量坐标系中的坐标    
    void clear()
    {
        //iSensorScanFreq = 0;//这里不能清零，只有在LMS扫描线程中会初始化该值。所以不可清零。
        vEncoder.clear();
        vvPointsCloud.clear();
    }
};

