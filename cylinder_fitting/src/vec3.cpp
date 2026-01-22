#include "vec3.h"

Vec3::Vec3(double x, double y, double z) {
    this->x = x;
    this->y = y;
    this->z = z;
}

Vec3::Vec3(const Vec3 &right) {
    this->x = right.x;
    this->y = right.y;
    this->z = right.z;
}

/**
 * 
 * @param right 
 * @return 
 */
Vec3& Vec3::operator=(const Vec3 &right) {
    if (this != &right) {
        this->x = right.x;
        this->y = right.y;
        this->z = right.z;
    }
    return *this;
}

// 乘以标量
Vec3 operator*(const Vec3 &left, double scalar) {
    Vec3 ret(left.x * scalar, left.y * scalar, left.z * scalar);
    return ret;
}

Vec3 operator*(double scalar, const Vec3 &left) {
    Vec3 ret(left.x * scalar, left.y * scalar, left.z * scalar);
    return ret;
}

// 点乘
Vec3 operator*(const Vec3 &left, const Vec3 &right) {
    Vec3 ret(left.x * right.x, left.y * right.y, left.z * right.z);
    return ret;
}

// 加法
Vec3 operator+(const Vec3 &left, const Vec3 &right) {
    Vec3 ret(left.x + right.x, left.y + right.y, left.z + right.z);
    return ret;
}

Vec3 operator+(const double &left, const Vec3 &right) {
    return Vec3(left + right.x, left + right.y, left + right.z);
}

Vec3 operator+(const Vec3 &left, const double &right) {
    return right + left;
}

Vec3 operator-(const double &left, const Vec3 &right) {
    Vec3 ret(left - right.x, left - right.y, left - right.z);
    return ret;
}

Vec3 operator-(const Vec3 &left, const double &right) {
    return right - left;
}

// 减法
Vec3 operator-(const Vec3 &left, const Vec3 &right) {
    Vec3 ret(left.x - right.x, left.y - right.y, left.z - right.z);
    return ret;
}
