#include "algorithm.h"
#include <memory>
#include <cmath>
#include <algorithm>
#include <random>
#include <numeric>
#include <vector>
#include <mutex>
#include "vec3.h"

std::mutex g_mut;

using namespace Algo;

const double pi = 4.0 * std::atan(1.0);

Algorithm *Algorithm::_my = nullptr;

Algorithm *Algorithm::createInstance() {
    if (nullptr == _my) {
        std::lock_guard<std::mutex> lock(g_mut);
        {
            if (nullptr == _my) {
                _my = new Algorithm;
            }
        }
    }
    return _my;
}

Algorithm::Algorithm() = default;

Algorithm::~Algorithm() = default;

void Algorithm::destroy() {
    if (nullptr != _my) {
        std::lock_guard<std::mutex> lock(g_mut);
        {
            if (nullptr != _my) {
                delete _my;
                _my = nullptr;
            }
        }
    }
}

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
bool Algorithm::robustFitCylinder(const Vec3 *points, const int pointNum,
                                  double *out_models) {
    std::unique_ptr<Vec3[]> x_points = std::make_unique<Vec3[]>(pointNum);
    const int angle_num = 3;
    double start_angle[][angle_num] = {{0,        0},
                                       {pi / 2.0, 0},
                                       {pi / 2.0, pi / 2.0}};

    Vec3 mu; // 均值
    _preprocess_data(points, pointNum, x_points.get(), mu);

    // 从不同角度拟合圆柱体
    double best_score = std::numeric_limits<double>::max();
    Vec3 best_dir;

    std::function<double(const Vec3 &)> fun = std::bind(&Algorithm::_G, *this,
                                                        std::placeholders::_1, x_points.get(), pointNum);

    for (auto & i : start_angle) {
        Vec3 dire = _direction(i[0], i[1]);
        //Vec3 dire = _direction(start_angle[1][0], start_angle[1][1]);
        double score = 0;
        Vec3 best_direction = _NelderMead(fun, dire, score);
        if (score < best_score) {
            best_score = score;
            best_dir = best_direction;
        }
    }

    Vec3 cen;
    double r;
    _C(best_dir, x_points.get(), pointNum, cen);
    _R(best_dir, x_points.get(), pointNum, r);

    out_models[0] = cen.x + mu.x;
    out_models[1] = cen.y + mu.y;
    out_models[2] = cen.z + mu.z;
    out_models[3] = best_dir.x;
    out_models[4] = best_dir.y;
    out_models[5] = best_dir.z;
    out_models[6] = r;
    return true;
}


// 叉乘
Vec3 Algorithm::cross(const Vec3 &left, const Vec3 &right) {
    Vec3 ret;
    ret.x = left.y * right.z - left.z * right.y;
    ret.y = right.x * left.z - left.x * right.z;
    ret.z = left.x * right.y - right.x * left.y;
    return ret;
}

// 取模
double Algorithm::norm(const Vec3 &vec) {
    Vec3 ret = vec * vec;
    return std::sqrt(ret.x + ret.y + ret.z);
}

bool Algorithm::computeNormal(const Vec3 **const points, const int *size,
                              Vec3 **out_normals) {
    bool res = true;
    const int m = size[0]; // 行数
    const int n = size[1]; // 列数
    const int minSize = 3;
    // 填充边界
    std::unique_ptr<Vec3[]> h0 = std::make_unique<Vec3[]>(n);
    std::unique_ptr<Vec3[]> he = std::make_unique<Vec3[]>(n);
    std::unique_ptr<Vec3[]> c0 = std::make_unique<Vec3[]>(m);
    std::unique_ptr<Vec3[]> ce = std::make_unique<Vec3[]>(m);

    if (m < minSize || n < minSize) {
        res = false;
        return res;
    }

    // 填充行
    for (size_t i = 0; i < n; ++i) {
        double x0 = 3.0 * points[0][i].x - 3.0 * points[1][i].x + points[2][i].x;
        double y0 = 3.0 * points[0][i].y - 3.0 * points[1][i].y + points[2][i].y;
        double z0 = 3.0 * points[0][i].z - 3.0 * points[1][i].z + points[2][i].z;
        h0[i] = Vec3(x0, y0, z0);
        double x1 = 3.0 * points[m - 1][i].x - 3.0 * points[m - 2][i].x + points[m - 3][i].x;
        double y1 = 3.0 * points[m - 1][i].y - 3.0 * points[m - 2][i].y + points[m - 3][i].y;
        double z1 = 3.0 * points[m - 1][i].z - 3.0 * points[m - 2][i].z + points[m - 3][i].z;
        he[i] = Vec3(x1, y1, z1);
    }
    // 填充列
    for (size_t i = 0; i < m; ++i) {
        double x0 = 3.0 * points[i][0].x - 3.0 * points[i][1].x + points[i][2].x;
        double y0 = 3.0 * points[i][0].y - 3.0 * points[i][1].y + points[i][2].y;
        double z0 = 3.0 * points[i][0].z - 3.0 * points[i][1].z + points[i][2].z;
        c0[i] = Vec3(x0, y0, z0);
        double x1 = 3.0 * points[i][n - 1].x - 3.0 * points[i][n - 2].x + points[i][n - 3].x;
        double y1 = 3.0 * points[i][n - 1].y - 3.0 * points[i][n - 2].y + points[i][n - 3].y;
        double z1 = 3.0 * points[i][n - 1].z - 3.0 * points[i][n - 2].z + points[i][n - 3].z;
        ce[i] = Vec3(x1, y1, z1);
    }

    const double esp = 1.0e-15;
    for (size_t i = 0; i < m; ++i) {
        double nx = 0.0, ny = 0.0, nz = 0.0;

        for (size_t j = 0; j < n; ++j) {
            double x0 = 0.0, y0 = 0.0, z0 = 0.0;
            double x1 = 0.0, y1 = 0.0, z1 = 0.0;
            if (0LL == i) {
                x1 = 0.5 * (points[1LL][j].x - h0[j].x);
                y1 = 0.5 * (points[1LL][j].y - h0[j].y);
                z1 = 0.5 * (points[1LL][j].z - h0[j].z);
            } else if (m - 1LL == i) {
                x1 = 0.5 * (he[j].x - points[m - 2LL][j].x);
                y1 = 0.5 * (he[j].y - points[m - 2LL][j].y);
                z1 = 0.5 * (he[j].z - points[m - 2LL][j].z);
            } else {
                x1 = 0.5 * (points[i + 1LL][j].x - points[i - 1LL][j].x);
                y1 = 0.5 * (points[i + 1LL][j].y - points[i - 1LL][j].y);
                z1 = 0.5 * (points[i + 1LL][j].z - points[i - 1LL][j].z);
            }
            if (0LL == j) {
                x0 = 0.5 * (c0[i].x - points[i][1LL].x);
                y0 = 0.5 * (c0[i].y - points[i][1LL].y);
                z0 = 0.5 * (c0[i].z - points[i][1LL].z);
            } else if (n - 1LL == j) {
                x0 = 0.5 * (points[i][n - 2LL].x - ce[i].x);
                y0 = 0.5 * (points[i][n - 2LL].y - ce[i].y);
                z0 = 0.5 * (points[i][n - 2LL].z - ce[i].z);

            } else {
                x0 = 0.5 * (points[i][j + 1LL].x - points[i][j - 1LL].x);
                y0 = 0.5 * (points[i][j + 1LL].y - points[i][j - 1LL].y);
                z0 = 0.5 * (points[i][j + 1LL].z - points[i][j - 1LL].z);
            }
            nx = -(y0 * z1 - z0 * y1);
            ny = -(z0 * x1 - x0 * z1);
            nz = -(x0 * y1 - y0 * x1);
            // 归一化
            double mag = std::sqrt(nx * nx + ny * ny + nz * nz);
            if (mag < esp) {
                mag = 1.0;
            }
            out_normals[i][j] = Vec3(nx / mag, ny / mag, nz / mag);
        }
    }
    return res;
}

bool Algorithm::fitCylinder(const Vec3 *const points,
                            const Vec3 *const normals, const int pointNum, const double maxR,
                            double *out_models, bool *inliners) {
    bool ret = true;
    int cur_iterator = 0; // 当前迭代次数
    double bestDis = maxR * 10; // 最优距离
    const int sampleSize = 2;
    int allIterator = 1000;

    if (nullptr == points || nullptr == normals || pointNum < 3) {
        return !ret;
    }

    // 开始迭代查找
    while (cur_iterator < allIterator) {
        // 随机选择两个点进行模型拟合
        std::random_device rndDevice;
        std::mt19937 eng(rndDevice());// 随机数引擎
        int rand1 = 0, rand2 = 0;
        std::uniform_real_distribution<double> suiji(0.0, pointNum - 1.0);
        while (rand1 == rand2) {
            rand1 = static_cast<int>(suiji(eng));
            rand2 = static_cast<int>(suiji(eng));
        }
        Vec3 p1 = points[rand1];
        Vec3 n1 = normals[rand1];
        Vec3 p2 = points[rand2];
        Vec3 n2 = normals[rand2];

        bool ok = _fitCylinder(p1, n1, p2, n2, out_models);

        // 评估模型结果
        if (ok) {
            // 计算所有点到模型的距离
            std::unique_ptr<double[]> dis = std::make_unique<double[]>(pointNum);
            _evalCylinder(out_models, points, pointNum, maxR, dis.get());
            int inNum = 0;
            for (int i = 0; i < pointNum; ++i) {
                if (dis[i] < maxR) {
                    inliners[i] = true;
                    inNum++;
                } else {
                    inliners[i] = false;
                }

            }
            double sum = std::accumulate(dis.get(), dis.get() + pointNum, 0.0);
            if (sum < bestDis) {
                bestDis = sum;
                // 计算迭代次数
                int num = _computeLoopNumber(sampleSize, pointNum, inNum);
                allIterator = std::min(1000, num);
            }
        }
        cur_iterator++;
    }
    _convertToFiniteCylinderModel(out_models, points, pointNum, inliners);
    return ret;
}


void Algorithm::_preprocess_data(const Vec3 *const points, const int pointNum,
                                 Vec3 *x_points, Vec3 &m) {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    for (size_t i = 0; i < pointNum; ++i) {
        x += points[i].x;
        y += points[i].y;
        z += points[i].z;
    }

    m.x = x / pointNum;
    m.y = y / pointNum;
    m.z = z / pointNum;

    for (size_t i = 0; i < pointNum; ++i) {
        x_points[i].x = points[i].x - m.x;
        x_points[i].y = points[i].y - m.y;
        x_points[i].z = points[i].z - m.z;
    }
}

Vec3 Algorithm::_direction(double theta, double phi) {
    Vec3 res;
    res.x = std::cos(phi) * std::sin(theta);
    res.y = std::sin(phi) * std::sin(theta);
    res.z = std::cos(theta);

    return res;
}

double Algorithm::_G(const Vec3 &w, Vec3 *points, const int pointNum) {
    double proj[3][3] = {0};
    double *p[3];
    p[0] = proj[0];
    p[1] = proj[1];
    p[2] = proj[2];

    _projection_matrix(w, p);
    Vec3 ys;
    Vec3 ys2(0, 0, 0);
    double u = 0.0;
    // 矩阵A元素
    double A[3][3] = {0};

    for (size_t i = 0; i < pointNum; ++i) {
        ys.x = proj[0][0] * points[i].x +
               proj[0][1] * points[i].y +
               proj[0][2] * points[i].z;

        ys.y = proj[1][0] * points[i].x +
               proj[1][1] * points[i].y +
               proj[1][2] * points[i].z;

        ys.z = proj[2][0] * points[i].x +
               proj[2][1] * points[i].y +
               proj[2][2] * points[i].z;

        // Ys累积和
        ys2.x += (ys.x * ys.x + ys.y * ys.y + ys.z * ys.z) * ys.x;
        ys2.y += (ys.x * ys.x + ys.y * ys.y + ys.z * ys.z) * ys.y;
        ys2.z += (ys.x * ys.x + ys.y * ys.y + ys.z * ys.z) * ys.z;

        // 矩阵A
        A[0][0] += ys.x * ys.x;
        A[0][1] += ys.x * ys.y;
        A[0][2] += ys.x * ys.z;

        A[1][0] += ys.y * ys.x;
        A[1][1] += ys.y * ys.y;
        A[1][2] += ys.y * ys.z;

        A[2][0] += ys.z * ys.x;
        A[2][1] += ys.z * ys.y;
        A[2][2] += ys.z * ys.z;

        u += ys.x * ys.x + ys.y * ys.y + ys.z * ys.z;
    }
    u /= pointNum;

    double A_hat[3][3];

    // w
    double skew_matrix[3][3] = {{0,    -w.z, w.y},
                                {w.z,  0,    -w.x},
                                {-w.y, w.x,  0}};
    // w^T
    double skew_matrix_transpose[3][3] = {{0,    w.z,  -w.y},
                                          {-w.z, 0,    w.x},
                                          {w.y,  -w.x, 0}};

    // A * skew_matrix_transpose
    double tmp[3][3];
    tmp[0][0] = A[0][0] * skew_matrix_transpose[0][0] +
                A[0][1] * skew_matrix_transpose[1][0] +
                A[0][2] * skew_matrix_transpose[2][0];

    tmp[0][1] = A[0][0] * skew_matrix_transpose[0][1] +
                A[0][1] * skew_matrix_transpose[1][1] +
                A[0][2] * skew_matrix_transpose[2][1];

    tmp[0][2] = A[0][0] * skew_matrix_transpose[0][2] +
                A[0][1] * skew_matrix_transpose[1][2] +
                A[0][2] * skew_matrix_transpose[2][2];

    tmp[1][0] = A[1][0] * skew_matrix_transpose[0][0] +
                A[1][1] * skew_matrix_transpose[1][0] +
                A[1][2] * skew_matrix_transpose[2][0];

    tmp[1][1] = A[1][0] * skew_matrix_transpose[0][1] +
                A[1][1] * skew_matrix_transpose[1][1] +
                A[1][2] * skew_matrix_transpose[2][1];

    tmp[1][2] = A[1][0] * skew_matrix_transpose[0][2] +
                A[1][1] * skew_matrix_transpose[1][2] +
                A[1][2] * skew_matrix_transpose[2][2];

    tmp[2][0] = A[2][0] * skew_matrix_transpose[0][0] +
                A[2][1] * skew_matrix_transpose[1][0] +
                A[2][2] * skew_matrix_transpose[2][0];

    tmp[2][1] = A[2][0] * skew_matrix_transpose[0][1] +
                A[2][1] * skew_matrix_transpose[1][1] +
                A[2][2] * skew_matrix_transpose[2][1];

    tmp[2][2] = A[2][0] * skew_matrix_transpose[0][2] +
                A[2][1] * skew_matrix_transpose[1][2] +
                A[2][2] * skew_matrix_transpose[2][2];

    // skew_matrix * (A * skew_matrix_transpose)
    // 第0行
    A_hat[0][0] = skew_matrix[0][0] * tmp[0][0] +
                  skew_matrix[0][1] * tmp[1][0] +
                  skew_matrix[0][2] * tmp[2][0];

    A_hat[0][1] = skew_matrix[0][0] * tmp[0][1] +
                  skew_matrix[0][1] * tmp[1][1] +
                  skew_matrix[0][2] * tmp[2][1];

    A_hat[0][2] = skew_matrix[0][0] * tmp[0][2] +
                  skew_matrix[0][1] * tmp[1][2] +
                  skew_matrix[0][2] * tmp[2][2];
    // 第1行
    A_hat[1][0] = skew_matrix[1][0] * tmp[0][0] +
                  skew_matrix[1][1] * tmp[1][0] +
                  skew_matrix[1][2] * tmp[2][0];

    A_hat[1][1] = skew_matrix[1][0] * tmp[0][1] +
                  skew_matrix[1][1] * tmp[1][1] +
                  skew_matrix[1][2] * tmp[2][1];

    A_hat[1][2] = skew_matrix[1][0] * tmp[0][2] +
                  skew_matrix[1][1] * tmp[1][2] +
                  skew_matrix[1][2] * tmp[2][2];

    // 第2行
    A_hat[2][0] = skew_matrix[2][0] * tmp[0][0] +
                  skew_matrix[2][1] * tmp[1][0] +
                  skew_matrix[2][2] * tmp[2][0];

    A_hat[2][1] = skew_matrix[2][0] * tmp[0][1] +
                  skew_matrix[2][1] * tmp[1][1] +
                  skew_matrix[2][2] * tmp[2][1];

    A_hat[2][2] = skew_matrix[2][0] * tmp[0][2] +
                  skew_matrix[2][1] * tmp[1][2] +
                  skew_matrix[2][2] * tmp[2][2];

    Vec3 v, trace;
    trace.x = A_hat[0][0] * A[0][0] + A_hat[0][1] * A[1][0] + A_hat[0][2] * A[2][0];
    trace.y = A_hat[1][0] * A[0][1] + A_hat[1][1] * A[1][1] + A_hat[1][2] * A[2][1];
    trace.z = A_hat[2][0] * A[0][2] + A_hat[2][1] * A[1][2] + A_hat[2][2] * A[2][2];

    v.x = A_hat[0][0] * ys2.x + A_hat[0][1] * ys2.y + A_hat[0][2] * ys2.z;
    v.y = A_hat[1][0] * ys2.x + A_hat[1][1] * ys2.y + A_hat[1][2] * ys2.z;
    v.z = A_hat[2][0] * ys2.x + A_hat[2][1] * ys2.y + A_hat[2][2] * ys2.z;

    v.x /= (trace.x + trace.y + trace.z);
    v.y /= (trace.x + trace.y + trace.z);
    v.z /= (trace.x + trace.y + trace.z);

    double res = 0;
    for (size_t i = 0; i < pointNum; ++i) {
        ys.x = proj[0][0] * points[i].x +
               proj[0][1] * points[i].y +
               proj[0][2] * points[i].z;

        ys.y = proj[1][0] * points[i].x +
               proj[1][1] * points[i].y +
               proj[1][2] * points[i].z;

        ys.z = proj[2][0] * points[i].x +
               proj[2][1] * points[i].y +
               proj[2][2] * points[i].z;

        double yv = (ys.x * v.x + ys.y * v.y + ys.z * v.z);

        double yy = (ys.x * ys.x + ys.y * ys.y + ys.z * ys.z);

        res += (yy - u - 2 * yv) * (yy - u - 2 * yv);
    }
    return res;
}

Vec3 Algorithm::_NelderMead(std::function<double(const Vec3 &w)> fun, Vec3 &x0,
                            double &bestScore, const int maxIter, const double err) {
    const int N = 3;
    // 初始化
    std::vector<Vec3> X(N + 1, Vec3(0, 0, 0)); // 存放输入的x值
    std::vector<Vec3> Y(N + 1, Vec3(0, 0, 0)); // 存放对应的y值，其中，x表示y值，y表示其在数组中的下标，方便后面排序知道下标
    X[0] = x0;
    Y[0] = Vec3(fun(x0), 0);
    const double usual_delta = 0.05;
    const double zero_term_delta = 0.00025;
    // 构造N个初始值，与第0个元素一起组成N+1个初始值
    for (size_t i = 0; i < N; ++i) {
        Vec3 y = X[0];
        if (0 == i) {
            if (std::abs(y.x) < 1.0e-16) {
                y.x = zero_term_delta;
            } else {
                y.x = (1 + usual_delta) * y.x;
            }
        } else if (1 == i) {
            if (std::abs(y.y) < 1.0e-16) {
                y.y = zero_term_delta;
            } else {
                y.y = (1 + usual_delta) * y.y;
            }
        } else {
            if (std::abs(y.z) < 1.0e-16) {
                y.z = zero_term_delta;
            } else {
                y.z = (1 + usual_delta) * y.z;
            }
        }

        X[i + 1] = y;
    }

    // 升序排序
    std::function<void(std::vector<Vec3> &x, std::vector<Vec3> &y)> inner =
            [](std::vector<Vec3> &x, std::vector<Vec3> &y) {

                size_t n = y.size();

                std::sort(y.begin(), y.end(), [](const Vec3 &f, const Vec3 &s) {
                    if (f.x < s.x) {
                        return true;
                    } else {
                        return false;
                    }
                });

                std::vector<Vec3> tmp;

                for (size_t i = 0; i < n; ++i) {
                    tmp.push_back(x[static_cast<size_t>(y[i].y)]);
                }
                x.clear();
                x = tmp;
            };


    int func_evals = 4; // 函数计算次数
    int maxfun = 600; // 200 * 3(3元素)
    // 开始迭代求解最优值
    int iter = 0;
    const double tolx = 1.0e-4;
    const double tolf = 1.0e-4;

    const double rho = 1.0;
    const double chi = 2.0;
    const double phi = 0.5;
    const double sigma = 0.5;

    while (iter < maxIter) {
        // 计算Y值，并排序
        for (size_t k = 0; k <= N; ++k) {
            Y[k] = Vec3(fun(X[k]), static_cast<double>(k));
        }
        inner(X, Y);
        if (norm(X[X.size() - 1LL] - X[0LL]) < err) {
            break;
        }

        // 求平均值
        Vec3 m(0, 0, 0);
        for (size_t j = 0; j <= N - 1LL; ++j) {
            m = m + X[j];
        }
        m = (1.0 / N) * m;
        Vec3 tt = 2.0 * m;
        Vec3 tt1 = (X[X.size() - 1]);
        Vec3 r = 2.0 * m - X[X.size() - 1];
        double fxr = fun(r);


        if (Y[0].x <= fxr && fxr <= Y[N - 1].x) // 第 4 步
        {
            X[N] = r;
            continue;
        } else if (fxr < Y[0].x) // 第 5 步
        {
            Vec3 s = m + 2 * (m - X[N]);
            double fsv = fun(s);
            if (fsv < fxr) {
                X[N] = s;
            } else {
                X[N] = r;
            }
            continue;
        } else if (fxr < Y[N].x) // 第 6 步
        {
            Vec3 c = m + (r - m) * 0.5;
            double fcv = fun(c);
            if (fcv < fxr) {
                X[N] = c;
                continue;
            }
        } else // 第 7 步
        {
            Vec3 c = m + (X[N] - m) * 0.5;
            double fcv = fun(c);
            if (fcv < Y[N].x) {
                X[N] = c;
                continue;
            }
        }
        // 第 8 步
        for (size_t k = 1; k <= N; ++k) {
            X[k] = X[0] + (X[k] - X[0]) * 0.5;
        }
        iter++;
    }

    Vec3 res = X[0];
    bestScore = fun(res);
    return res;
}

void Algorithm::_projection_matrix(const Vec3 &w, double **out3x3) {
    out3x3[0][0] = 1.0 - w.x * w.x;
    out3x3[0][1] = -w.x * w.y;
    out3x3[0][2] = -w.x * w.z;

    out3x3[1][0] = -w.y * w.x;
    out3x3[1][1] = 1.0 - w.y * w.y;
    out3x3[1][2] = -w.y * w.z;

    out3x3[2][0] = -w.z * w.x;
    out3x3[2][1] = -w.z * w.y;
    out3x3[2][2] = 1.0 - w.z * w.z;
}

void Algorithm::_C(Vec3 &w, const Vec3 *points, const int pointNum, Vec3 &cen) {
    double **proj = new double *[3];
    for (int i = 0; i < 3; ++i) {
        proj[i] = new double[3];
    }

    _projection_matrix(w, proj);
    Vec3 ys;
    Vec3 ys2(0, 0, 0); // Ys的累计和
    double u = 0.0, u2 = 0.0;
    double A[3][3] = {0};

    for (size_t i = 0; i < pointNum; ++i) {
        ys.x = proj[0][0] * points[i].x +
               proj[0][1] * points[i].y +
               proj[0][2] * points[i].z;

        ys.y = proj[1][0] * points[i].x +
               proj[1][1] * points[i].y +
               proj[1][2] * points[i].z;

        ys.z = proj[2][0] * points[i].x +
               proj[2][1] * points[i].y +
               proj[2][2] * points[i].z;
        // Ys累积和
        ys2.x += (ys.x * ys.x + ys.y * ys.y + ys.z * ys.z) * ys.x;
        ys2.y += (ys.x * ys.x + ys.y * ys.y + ys.z * ys.z) * ys.y;
        ys2.z += (ys.x * ys.x + ys.y * ys.y + ys.z * ys.z) * ys.z;
        // 矩阵A
        A[0][0] += ys.x * ys.x;
        A[0][1] += ys.x * ys.y;
        A[0][2] += ys.x * ys.z;

        A[1][0] += ys.y * ys.x;
        A[1][1] += ys.y * ys.y;
        A[1][2] += ys.y * ys.z;

        A[2][0] += ys.z * ys.x;
        A[2][1] += ys.z * ys.y;
        A[2][2] += ys.z * ys.z;
    }

    double A_hat[3][3];

    // w
    double skew_matrix[3][3] = {{0,    -w.z, w.y},
                                {w.z,  0,    -w.x},
                                {-w.y, w.x,  0}};
    // w^T
    double skew_matrix_transpose[3][3] = {{0,    w.z,  -w.y},
                                          {-w.z, 0,    w.x},
                                          {w.y,  -w.x, 0}};

    // A * skew_matrix_transpose
    double tmp[3][3];
    tmp[0][0] = A[0][0] * skew_matrix_transpose[0][0] +
                A[0][1] * skew_matrix_transpose[1][0] +
                A[0][2] * skew_matrix_transpose[2][0];

    tmp[0][1] = A[0][0] * skew_matrix_transpose[0][1] +
                A[0][1] * skew_matrix_transpose[1][1] +
                A[0][2] * skew_matrix_transpose[2][1];

    tmp[0][2] = A[0][0] * skew_matrix_transpose[0][2] +
                A[0][1] * skew_matrix_transpose[1][2] +
                A[0][2] * skew_matrix_transpose[2][2];

    tmp[1][0] = A[1][0] * skew_matrix_transpose[0][0] +
                A[1][1] * skew_matrix_transpose[1][0] +
                A[1][2] * skew_matrix_transpose[2][0];

    tmp[1][1] = A[1][0] * skew_matrix_transpose[0][1] +
                A[1][1] * skew_matrix_transpose[1][1] +
                A[1][2] * skew_matrix_transpose[2][1];

    tmp[1][2] = A[1][0] * skew_matrix_transpose[0][2] +
                A[1][1] * skew_matrix_transpose[1][2] +
                A[1][2] * skew_matrix_transpose[2][2];

    tmp[2][0] = A[2][0] * skew_matrix_transpose[0][0] +
                A[2][1] * skew_matrix_transpose[1][0] +
                A[2][2] * skew_matrix_transpose[2][0];

    tmp[2][1] = A[2][0] * skew_matrix_transpose[0][1] +
                A[2][1] * skew_matrix_transpose[1][1] +
                A[2][2] * skew_matrix_transpose[2][1];

    tmp[2][2] = A[2][0] * skew_matrix_transpose[0][2] +
                A[2][1] * skew_matrix_transpose[1][2] +
                A[2][2] * skew_matrix_transpose[2][2];

    // skew_matrix * (A * skew_matrix_transpose)
    // 第0行
    A_hat[0][0] = skew_matrix[0][0] * tmp[0][0] +
                  skew_matrix[0][1] * tmp[1][0] +
                  skew_matrix[0][2] * tmp[2][0];

    A_hat[0][1] = skew_matrix[0][0] * tmp[0][1] +
                  skew_matrix[0][1] * tmp[1][1] +
                  skew_matrix[0][2] * tmp[2][1];

    A_hat[0][2] = skew_matrix[0][0] * tmp[0][2] +
                  skew_matrix[0][1] * tmp[1][2] +
                  skew_matrix[0][2] * tmp[2][2];
    // 第1行
    A_hat[1][0] = skew_matrix[1][0] * tmp[0][0] +
                  skew_matrix[1][1] * tmp[1][0] +
                  skew_matrix[1][2] * tmp[2][0];

    A_hat[1][1] = skew_matrix[1][0] * tmp[0][1] +
                  skew_matrix[1][1] * tmp[1][1] +
                  skew_matrix[1][2] * tmp[2][1];

    A_hat[1][2] = skew_matrix[1][0] * tmp[0][2] +
                  skew_matrix[1][1] * tmp[1][2] +
                  skew_matrix[1][2] * tmp[2][2];

    // 第2行
    A_hat[2][0] = skew_matrix[2][0] * tmp[0][0] +
                  skew_matrix[2][1] * tmp[1][0] +
                  skew_matrix[2][2] * tmp[2][0];

    A_hat[2][1] = skew_matrix[2][0] * tmp[0][1] +
                  skew_matrix[2][1] * tmp[1][1] +
                  skew_matrix[2][2] * tmp[2][1];

    A_hat[2][2] = skew_matrix[2][0] * tmp[0][2] +
                  skew_matrix[2][1] * tmp[1][2] +
                  skew_matrix[2][2] * tmp[2][2];

    Vec3 v, trace;
    trace.x = A_hat[0][0] * A[0][0] + A_hat[0][1] * A[1][0] + A_hat[0][2] * A[2][0];
    trace.y = A_hat[1][0] * A[0][1] + A_hat[1][1] * A[1][1] + A_hat[1][2] * A[2][1];
    trace.z = A_hat[2][0] * A[0][2] + A_hat[2][1] * A[1][2] + A_hat[2][2] * A[2][2];

    cen.x = A_hat[0][0] * ys2.x + A_hat[0][1] * ys2.y + A_hat[0][2] * ys2.z;
    cen.y = A_hat[1][0] * ys2.x + A_hat[1][1] * ys2.y + A_hat[1][2] * ys2.z;
    cen.z = A_hat[2][0] * ys2.x + A_hat[2][1] * ys2.y + A_hat[2][2] * ys2.z;

    cen.x /= (trace.x + trace.y + trace.z);
    cen.y /= (trace.x + trace.y + trace.z);
    cen.z /= (trace.x + trace.y + trace.z);

    // 释放二维数组
    for (int i = 0; i < 3; ++i) {
        delete[] proj[i];
    }
    delete[] proj;
}

void Algorithm::_R(Vec3 &w, const Vec3 *points, const int pointNum, double &r) {
    double proj[3][3] = {0};
    double *p[3];
    p[0] = proj[0];
    p[1] = proj[1];
    p[2] = proj[2];
    r = 0;
    _projection_matrix(w, p);
    Vec3 cen;
    _C(w, points, pointNum, cen);
    Vec3 ys;

    for (size_t i = 0; i < pointNum; ++i) {
        Vec3 c_x;
        c_x.x = cen.x - points[i].x;
        c_x.y = cen.y - points[i].y;
        c_x.z = cen.z - points[i].z;

        Vec3 p_c_x;
        p_c_x.x = p[0][0] * c_x.x + p[0][1] * c_x.y + p[0][2] * c_x.z;
        p_c_x.y = p[1][0] * c_x.x + p[1][1] * c_x.y + p[1][2] * c_x.z;
        p_c_x.z = p[2][0] * c_x.x + p[2][1] * c_x.y + p[2][2] * c_x.z;

        r += (c_x.x * p_c_x.x) + (c_x.y * p_c_x.y) + (c_x.z * p_c_x.z);
    }
    r /= pointNum;
    r = std::sqrt(r);
}

bool Algorithm::_brent(const std::function<double(double)>& f,
                       const double *interval, double *root, const int maxIter, const double tol) {
    double a = interval[0];
    double b = interval[1];

    double fa = f(a);
    double fb = f(b);
    if (fa * fb > 0.0) {
        return false;
    }
    if (std::abs(fa) < tol) {
        *root = a;
        return true;
    }
    if (std::abs(fb) < tol) {
        *root = b;
        return true;
    }

    if (std::abs(fa) < std::abs(fb)) {
        std::swap(a, b);
        std::swap(fa, fb);
    }
    double c = a;
    double fc = fa;

    double s = 0.0;
    double d = 0.0;
    double fs = f(s); // f(0)
    bool m_flag = true;

    if (std::abs(fs) < tol) {
        *root = s;
        return true;
    }

    for (int i = 0; i < maxIter; ++i) {
        if (std::abs(fa - fc) > 1.0e-6 && std::abs(fb - fc) > 1.0e-6) {
            // 逆向二次插值
            double t1 = (a * fb * fc) / ((fa - fb) * (fa - fc));
            double t2 = (b * fa * fc) / ((fb - fa) * (fb - fc));
            double t3 = (c * fa * fb) / ((fc - fa) * (fc - fb));
            s = t1 + t2 + t3;
        } else {
            // 割线法
            s = b - fb * (b - a) / (fb - fa);
        }

        if ((s < (3 * a + b) * 0.25 || s > b) ||
            (m_flag && (std::abs(s - b) >= 0.5 * std::abs(b - c))) ||
            (!m_flag && (std::abs(s - b) >= 0.5 * std::abs(c - d))) ||
            (m_flag && (std::abs(b - c) < tol)) ||
            (!m_flag && (std::abs(c - d) < tol))) {
            // 二分法
            s = 0.5 * (a + b);
            m_flag = true;
        } else {
            m_flag = false;
        }
        fs = f(s);
        d = c;
        c = b;
        fc = fb;
        if (fa * fs < 0.0) {
            b = s;
            fb = fs;
        } else {
            a = s;
            fa = fs;
        }
        if (std::abs(fa) < std::abs(fb)) {
            std::swap(a, b);
            std::swap(fa, fb);
        }
        // 判断是否收敛
        if (std::abs(fb) < tol || std::abs(fs) < tol || std::abs(a - b) < tol) {
            *root = b;
            return true;
        }
    }
    
    return false;
}

bool Algorithm::_fitCylinder(const Vec3 &p1, const Vec3 &n1,
                             const Vec3 &p2, const Vec3 &n2, double *out) {
    bool res = false;
    const double esp = 1.0e-20;
    Vec3 w = ((p1 + n1) - p2);
    Vec3 dot_n1_n1 = n1 * n1;
    Vec3 dot_n1_n2 = n1 * n2;
    Vec3 dot_n2_n2 = n2 * n2;
    Vec3 dot_n1_w = n1 * w;
    Vec3 dot_n2_w = n2 * w;
    double a = dot_n1_n1.x + dot_n1_n1.y + dot_n1_n1.z;
    double b = dot_n1_n2.x + dot_n1_n2.y + dot_n1_n2.z;
    double c = dot_n2_n2.x + dot_n2_n2.y + dot_n2_n2.z;
    double d = dot_n1_w.x + dot_n1_w.y + dot_n1_w.z;
    double e = dot_n2_w.x + dot_n2_w.y + dot_n2_w.z;
    double D = a * c - b * b;
    double s = 0.0;
    double t = 0.0;

    if (D < 1.0e-5) {
        if (b > c) {
            if (std::abs(b) < esp) {
                return res;
            }
            t = d / b;
        } else {
            if (std::abs(c) < esp) {
                return res;
            }
            t = e / c;
        }
    } else {
        if (std::abs(D) < esp) {
            return res;
        }
        s = (b * e - c * d) / D;
        t = (a * e - b * d) / D;
    }
    // 计算圆柱轴上点
    Vec3 p0 = p1 + n1 + n1 * s;
    // 计算圆柱轴方向向量--归一化
    Vec3 dp = p2 + t * n2 - p0;
    double ndp = norm(dp);
    if (ndp < esp) {
        return res;
    }
    dp = dp * (1.0 / ndp);// 归一化圆柱体轴方向向量
    // 计算圆柱体半径
    Vec3 p1p0 = p1 - p0;
    Vec3 p2p1 = dp;
    Vec3 cr = cross(p1p0, p2p1);
    double r = norm(cr);

    out[0] = p0.x;
    out[1] = p0.y;
    out[2] = p0.z;

    out[3] = dp.x;
    out[4] = dp.y;
    out[5] = dp.z;
    out[6] = r;
    res = true;
    return res;
}

void Algorithm::_evalCylinder(const double *model,
                              const Vec3 *points, const int pointNum, const double maxR, double *dis) {
    // 计算每个点到圆柱轴的距离
    Vec3 p0(model[0], model[1], model[2]);

    Vec3 p2p1(model[3], model[4], model[5]);

    double r = model[6];
    for (int i = 0; i < pointNum; ++i) {
        Vec3 p0p1(points[i] - p0);
        Vec3 cr = cross(p0p1, p2p1);
        double d = std::abs(norm(cr) - r);
        if (d > maxR) {
            d = maxR;
        }
        dis[i] = d;
    }
}

int Algorithm::_computeLoopNumber(const int sampSize, const int numPts, const int inNum) {
    int iterator_num = 0;
    const double confidence = 0.99; // 默认有99%的内点
    const double esp = 1.0e-4; // 0.01%
    // 计算随机取点为内点的概率
    double probility = std::pow(static_cast<double>(inNum) / numPts, sampSize);
    if (probility < confidence)// 有外部点，还需要继续迭代
    {
        double num = log10(1.0 - confidence); // 分子
        double den = log10(1.0 - probility); // 分母
        iterator_num = static_cast<int>(ceil(num / den));
    }
    return iterator_num;
}

void Algorithm::_convertToFiniteCylinderModel(double *model,
                                              const Vec3 *points, const int pointsNum, const bool *in_inliers) {
    Vec3 dp = Vec3(model[3], model[4], model[5]);// 圆柱轴线方向向量
    Vec3 p0 = Vec3(model[0], model[1], model[2]);// 圆柱轴上一点
    double r = model[6];// 圆柱半径
    std::vector<double> k;
    Vec3 u = p0 * dp;
    double con = u.x + u.y + u.z;
    for (int i = 0; i < pointsNum; ++i) {
        if (in_inliers[i]) {
            Vec3 tmp = points[in_inliers[i]] * dp;
            k.push_back(tmp.x + tmp.y + tmp.z - con);
        }
    }

    auto result = std::minmax_element(k.begin(), k.end());
    Vec3 pa = p0 + (*result.first) * dp;
    Vec3 pb = p0 + (*result.second) * dp;

    model[0] = pa.x;
    model[1] = pa.y;
    model[2] = pa.z;

    model[3] = pb.x;
    model[4] = pb.y;
    model[5] = pb.z;
}

bool Algorithm::fitLinear2D(const Vec3 *const points, const int pointNum,
                            const double threshold, double *out_models, bool *inliners) {
    // 初始化一些参数
    bool ret = true;
    int cur_iterator = 0; // 当前迭代次数
    double bestDis = 10000.0; // 最优距离
    const int sampleSize = 2;
    int allIterator = 1000;// 总的迭代次数
    int inNum = 0;
    while (cur_iterator < allIterator) {
        // 随机选择两个点进行模型拟合
        std::random_device rndDevice;
        std::mt19937 eng(rndDevice());// 随机数引擎
        int rand1 = 0, rand2 = 0;
        const int sampSize = 2;
        std::uniform_real_distribution<double> suiji(0.0, pointNum - 1.0);
        while (rand1 == rand2) {
            rand1 = static_cast<int>(suiji(eng));
            rand2 = static_cast<int>(suiji(eng));
        }
        std::unique_ptr<Vec3[]> samPoints =
                std::make_unique<Vec3[]>(sampSize);
        samPoints[0] = points[rand1];
        samPoints[1] = points[rand2];
        // 基于两点及其法向量拟合模型
        bool ok = _LeastSquareFitLinear(samPoints.get(), sampleSize, out_models);
        // 评估模型结果
        if (ok) {
            // 计算所有点到模型的距离
            std::unique_ptr<double[]> dis = std::make_unique<double[]>(pointNum);
            _evalLine(points, pointNum, threshold,
                      out_models, dis.get());
            // 统计内点

            for (size_t i = 0; i < pointNum; ++i) {
                if (dis[i] < threshold) {
                    inliners[i] = true;
                    inNum++;
                } else {
                    inliners[i] = false;
                }
            }
            double sum = std::accumulate(dis.get(), dis.get() + pointNum, 0.0);
            if (sum < bestDis) {
                bestDis = sum;
                // 计算迭代次数
                int num = _computeLoopNumber(sampleSize, pointNum, inNum);
                allIterator = std::min(1000, num);
            }
        }
        cur_iterator++;
    }
    // 找到了最优模型，使用最小二乘法重新评估模型参数
    std::unique_ptr<Vec3[]> pp = std::make_unique<Vec3[]>(inNum);
    int j = 0;
    for (int i = 0; i < pointNum; ++i) {
        if (inliners[i]) {
            pp[j] = points[i];
            j++;
        }
    }
    _LeastSquareFitLinear(pp.get(), sampleSize, out_models);
    return ret;
}

bool Algorithm::_LeastSquareFitLinear(const Vec3 *pointCloud,
                                      const int pointNum, double *model_para) {
    bool res = true;
    // 至少有2个点才能拟合直线

    if (pointNum < 2) {
        return false;
    }
    double A{0.0};
    double B{0.0};
    double C{0.0};
    double D{0.0};
    double meanx{0.0};
    double meany{0.0};
    double sigma_x{0.0};

    for (size_t i = 0; i < pointNum; ++i) {
        double x = pointCloud[i].x;
        double y = pointCloud[i].y;
        A += (x * x);
        B += x;
        C += (x * y);
        D += y;
    }
    meanx = B / pointNum;
    meany = D / pointNum;
    // 计算x方差
    for (size_t i = 0; i < pointNum; ++i) {
        double x = pointCloud[i].x;
        sigma_x += (x - meanx) * (x - meanx);
    }
    sigma_x = sigma_x / pointNum;
    double den = pointNum * A - B * B;
    if (sigma_x < 2) {
        model_para[0] = 1.0;
        model_para[1] = 0.0;
        model_para[2] = -meanx;
    } else if (std::abs(den) < 1.0e-6) {
        model_para[0] = 0.0;
        model_para[1] = 1.0;
        model_para[2] = -meany;
    } else {
        double k = (pointNum * C - B * D) / den;
        double b = (A * D - C * B) / den;
        model_para[0] = k;
        model_para[1] = -1;
        model_para[2] = b;
    }
    return res;
}

void Algorithm::_evalLine(const Vec3 *pointCloud, const int pointNum,
                          const double threshold_square, const double *const model_para, double *dis) {
    const double esp = 1.0e-6;
    if (std::abs(model_para[0]) < esp) // 平行于y轴
    {
        double y0 = -model_para[2] / model_para[1];
        for (size_t i = 0; i < pointNum; ++i) {
            double d = (pointCloud[i].y - y0) *
                       (pointCloud[i].y - y0);
            if (d > threshold_square) {
                d = threshold_square;
            }
            dis[i] = d;
        }
    } else if (std::abs(model_para[1]) < esp) // 平行于x轴
    {
        double x0 = -model_para[2] / model_para[0];
        for (size_t i = 0; i < pointNum; ++i) {
            double d = (pointCloud[i].x - x0) *
                       (pointCloud[i].x - x0);
            if (d > threshold_square) {
                d = threshold_square;
            }
            dis[i] = d;
        }
    } else {
        double k = -model_para[0] / model_para[1];
        double b = -model_para[2] / model_para[1];
        for (size_t i = 0; i < pointNum; ++i) {
            double y0 = k * pointCloud[i].x + b;
            double d = (pointCloud[i].y - y0) *
                       (pointCloud[i].y - y0);
            if (d > threshold_square) {
                d = threshold_square;
            }
            dis[i] = d;
        }
    }
}

