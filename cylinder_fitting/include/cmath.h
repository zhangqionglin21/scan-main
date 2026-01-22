#ifndef ALGORITHM_CMATH_H
#define ALGORITHM_CMATH_H

#include <limits>
#include <cmath>

/**
 * 提供部分扩展函数
 */
namespace CMath 
{
    /**
     * 返回PI
     * @return 
     */
    double Pi();

    /**
     * 弧度转角度
     * @param rad 
     * @return 
     */
    double r2d(double rad);
   
    /**
     * 角度转弧度
     * @param degree 
     * @return 
     */
    double d2r(double degree);
}

#endif //ALGORITHM_CMATH_H