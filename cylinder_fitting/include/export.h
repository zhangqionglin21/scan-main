#ifndef ALGORITHM_INTERFACE_H
#define ALGORITHM_INTERFACE_H

#include "vec3.h"

// 导出类不能以C的方式导出
// 因为C不支持类
#ifdef ALGORITHM_LIB_API
#else
// Windows上特有的导出函数在对象前使用ALGORITHM_LIB_API表示对应的符号在导出的dll中可见 TODO: 不能跨平台
#define ALGORITHM_LIB_API __declspec(dllexport)
#endif

using Vec3 = struct Vec3;
using uint = unsigned int;

namespace Algo {
    class ALGORITHM_LIB_API IAlgorithm {
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
        virtual bool robustFitCylinder(const Vec3 *points, int pointNum,
                                       double *out_models) = 0;

        /***************
        * 释放资源
        ***************/
        virtual void destroy() = 0;
    };
}


// 多线程安全的工厂函数
extern "C"
{
ALGORITHM_LIB_API Algo::IAlgorithm *createInterface();
}

#endif //ALGORITHM_INTERFACE_H