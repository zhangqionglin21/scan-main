#include "cmath.h"

/**
 * 返回PI
 * @return 
 */
double CMath::Pi() {
    return 4.0 * std::atan(1.0);
}

/**
 * 弧度转角度
 * @param rad 
 * @return 
 */
double CMath::r2d(double rad) {
    return 180.0 * rad / CMath::Pi();
}

/**
 * 角度转弧度
 * @param degree 
 * @return 
 */
double CMath::d2r(double degree) {
    return CMath::Pi() * degree / 180.0;
}