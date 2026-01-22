//
// Created by yujianhua on 2024/1/6.
//

#ifndef ALGORITHM_VEC3_H
#define ALGORITHM_VEC3_H


/**
 * 三维数据的基本操作
 * 如果是二维的，则第三维元素赋值0
 * 加减乘除四则运算符号一般用全局函数实现
 */
struct Vec3 {
    // 相乘
    friend Vec3 operator*(const Vec3 &left, const Vec3 &right);

    friend Vec3 operator*(const Vec3 &left, double scalar);

    friend Vec3 operator*(double scalar, const Vec3 &left);

    // 相减
    friend Vec3 operator-(const Vec3 &left, const Vec3 &right);

    friend Vec3 operator-(const double &left, const Vec3 &right);

    friend Vec3 operator-(const Vec3 &left, const double &right);

    // 相加
    friend Vec3 operator+(const Vec3 &left, const Vec3 &right);

    friend Vec3 operator+(const double &left, const Vec3 &right);

    friend Vec3 operator+(const Vec3 &left, const double &right);

    // 坐标
    double x;
    double y;
    double z;
    
    // 构造函数
    explicit Vec3(double x = 0.0, double y = 0.0, double z = 0.0);
    
    // 构造函数
    Vec3(const Vec3 &right);
    
    // 重载赋值
    Vec3 &operator=(const Vec3 &right);
};

#endif //ALGORITHM_VEC3_H
