#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <functional>
#include "export.h"

using namespace Algo;

/**
 * 具体圆柱体拟合的算法实现
 */
class Algorithm : public IAlgorithm {
public:
    // 全局算法实例
    static Algorithm *createInstance();

    // 析构函数
    ~Algorithm();

public:
    Algorithm();

protected:
    void destroy() override;

public:
    /***************
    * 稳定的拟合圆柱体
    *
    * 输入参数：
    * @points: 点云数据，向量格式
    * @pointNum: 点云个数
    *
    * 输出参数：
    * @out_models：7个元素的数组，由应用层预先分配内存
    * 其中，[0, 1, 2] 表示圆柱体中心点坐标，
    * [3, 4, 5]表示圆柱体轴线方向向量，
    * [6] 表示圆柱体半径
    ***************/
    bool robustFitCylinder(const Vec3 *points, int pointNum,
                           double *out_models) override;

private:
    /***************
    * 参考MATLAB的基于MSAC算法拟合圆柱体--结果很不稳定，差评
    * 
    * 输入参数：
    * @points：点云数据，向量格式
    * @normals： 点云法向量，向量格式，可调用PCL相关函数计算，也可使用computeNormal函数计算
    * @pointNum：点云个数
    * @maxR：圆柱体最大半径，作为初始值
    * 
    * 输出参数：
    * @out_models：7个元素的数组，由用用层分配内存
    * 其中，[0, 1, 2] 表示圆柱体中心点坐标，
    * [3, 4, 5]表示圆柱体轴线方向向量，
    * [6] 表示圆柱体半径
    * @inliners：内点标识，用于应用层判断哪些点分布在圆柱面上，由应用层分配内存
    ***************/
    bool fitCylinder(const Vec3 *points, const Vec3 *normals,
                     int pointNum, double maxR, double *out_models, bool *inliners);

    /***************
    * 计算点云法向量--点云必须是m*n的矩阵
    * 
    * 输入参数：
    * @points: 点云数据，m*n矩阵
    * @size: 2元素的数组，size[0]表示矩阵行数，size[1]表示矩阵列数
    * 
    * 输出参数：
    * @out_normals: 矩阵法向量，m*n矩阵，需应用层分配内存
    ***************/
    static bool computeNormal(const Vec3 **points, const int *size, Vec3 **out_normals);

    /***************
    * 拟合形如ax + by + c = 0的二维直线--Vec3的z坐标为0
    *
    * 输入参数：
    * @points: 点云数据，向量形式
    * @pointNum:点云数量
    * @threshold：采样点距离直线距离的阈值
    *
    * 输出参数：
    * @out_normals: 直线参数[a, b, c]，由应用层分配内存
    * @inliners: 内点标识，由应用层分配内存
    ***************/
    bool fitLinear2D(const Vec3 *points, int pointNum, double threshold, double *out_models, bool *inliners);

private:
    /***************
    * 两点及其法向量拟合圆柱体--MATLAB使用的方法
    * 
    * 输入参数：
    * @p1, min_max: 圆柱面上的任意两点
    * @n1, n2: min_min, p2两点对应的法向量
    * 
    * 输出参数：
    * @out：圆柱体7个参数数组，由应用层分配内存
    * 其中，[0, 1, 2] 表示圆柱体中心点坐标，
    * [3, 4, 5]表示圆柱体轴线方向向量，
    * [6] 表示圆柱体半径
    ***************/
    static bool _fitCylinder(const Vec3 &p1, const Vec3 &n1, const Vec3 &p2, const Vec3 &n2, double *out);

    /***************
    * 评估圆柱体模型好坏
    * 
    * 输入参数：
    * @model：圆柱体模型参数
    * 其中，[0, 1, 2] 表示圆柱体中心点坐标，
    * [3, 4, 5]表示圆柱体轴线方向向量，
    * [6] 表示圆柱体半径
    * @points：点云数据
    * @pointNum：点云个数
    * @maxR：圆柱体最大半径
    * 
    * 输出参数
    * @dis：数组指针，采样点到圆柱体的距离，元素个数为pointNum,由应用层分配内存
    ***************/
    static void _evalCylinder(const double *model, const Vec3 *points, int pointNum, double maxR, double *dis);

    /***************
    *  计算MSAC迭代次数
    *
    * 输入参数：
    * @sampSize：采样点个数
    * @numPts：点云个数
    * @inNum：内点个数
    *
    * return:
    * 返回MSAC需要迭代的次数
    ***************/
    static int _computeLoopNumber(int sampSize, int numPts, int inNum);

    /***************
    *  把点云投射到圆柱体轴线上
    *
    * 输入参数：
    * @model：圆柱体模型参数
    * 其中，[0, 1, 2] 表示圆柱体中心点坐标，
    * [3, 4, 5]表示圆柱体轴线方向向量，
    * [6] 表示圆柱体半径
    * @points：点云数据
    * @pointNum：点云个数
    * @in_inliers：内点标识
    ***************/
    static void _convertToFiniteCylinderModel(double *model,
                                              const Vec3 *points, int pointsNum, const bool *in_inliers);

    /***************
    *  最小二乘法拟合形如ax + by + c = 0的二维直线
    *
    * 输入参数：
    * @pointCloud：点云数据
    * @pointNum：点云个数
    * 
    * 输出参数：
    * @model_para：直线参数[a, b, c]
    ***************/
    static bool _LeastSquareFitLinear(const Vec3 *pointCloud, int pointNum,
                               double *model_para);

    /***************
    * 评估直线拟合结果的好坏
    *
    * 输入参数：
    * @pointCloud：点云数据
    * @pointNum：点云个数
    * @threshold_square：采样点到直线距离的阈值
    * @model_para：直线参数[a, b, c]
    * 
    * 输出参数：
    * @dis：数组指针，采样点到圆柱体的距离，元素个数为pointNum,由应用层分配内存
    ***************/
    static void _evalLine(const Vec3 *pointCloud, int pointNum,
                   double threshold_square, const double *model_para, double *dis);

    // 叉乘
    static Vec3 cross(const Vec3 &left, const Vec3 &right);

    // 取模
    static double norm(const Vec3 &vec);

    /***************
    * 计算圆柱体轴线方向
    *
    * 输入参数：
    * @theta：单位弧度
    * @phi：
    *
    * 输出参数：
    * 返回圆柱体方向向量
    ***************/
    static Vec3 _direction(double theta, double phi);

    /***************
    * 计算圆柱体方向向量投影矩阵
    *
    * 输入参数：
    * @w：圆柱体方向向量
    *
    * 输出参数：
    * @out3x3：圆柱体方向向量投影矩阵，3x3矩阵
    ***************/
    static void _projection_matrix(const Vec3 &w, double **out3x3);

    /***************
    * 计算圆柱体中心点坐标
    *
    * 输入参数：
    * @w：圆柱体方向向量
    * @points：点云数据
    * @pointNum：点云个数
    *
    * 输出参数：
    * @cen：圆柱体中心坐标
    ***************/
    static void _C(Vec3 &w, const Vec3 *points, int pointNum, Vec3 &cen);

    /***************
    * 计算圆柱体半径
    *
    * 输入参数：
    * @w：圆柱体方向向量
    * @points：点云数据
    * @pointNum：点云个数
    *
    * 输出参数：
    * @r：圆柱体半径
    ***************/
    static void _R(Vec3 &w, const Vec3 *points, int pointNum, double &r);

    /***************
    * 数据预处理
    *
    * 输入参数：
    * @points：点云数据
    * @pointNum：点云个数
    *
    * 输出参数：
    * @x_points：点云漂移后的值
    * @m：点云均值
    ***************/
    static void _preprocess_data(const Vec3 *points, int pointNum,
                          Vec3 *x_points, Vec3 &m);

    /***************
    * 圆柱体拟合误差代价函数
    *
    * 输入参数：
    * @w：圆柱体方向向量
    * @points：点云数据
    * @pointNum：点云个数
    *
    * 输出参数：
    * 返回误差代价
    ***************/
    double _G(const Vec3 &w, Vec3 *points, int pointNum);

    /***************
    * NelderMead算法，求多元函数的极值的单纯形算法--simplex
    * 该算法适用于只有一个极值点的情况，即单峰，如果有多个极值点，
    * 可能找到的是局部极值点，而不是全局极值点
    * 
    * @fun: 待求极值的函数对象
    ***************/
    static Vec3 _NelderMead(std::function<double(const Vec3 &w)> fun,
                     Vec3 &x0, double &bestScore,
                     int maxIter = 1000, double err = 1.0e-5);

    /***************
    * 布伦特求根算法--一维
    *
    * 输入参数：
    * @func：待求根的函数对象
    * @interval：2元素数组，包含根的区间
    * @maxIter：最大迭代次数
    * @tol：误差限
    *
    * 输出参数：
    * @root：零点对应的x坐标
    ***************/
    static bool _brent(const std::function<double(double)> &func, const double *interval, double *root,
                int maxIter = 1000, double tol = 1.0e-6);

private:
    static Algorithm *_my;
};

#endif // ALGORITHM_H