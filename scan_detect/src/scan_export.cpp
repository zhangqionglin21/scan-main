#include "scan_export.h"
#include "train_scan.h"
#include "car_scan.h"
#include "agv_scan.h"
#include "extensions.h"
#include "pcl/io/pcd_io.h"

/**
 * @brief 火车鞍座识别，单车厢？
 * @param cloud_in 输入点云
 * @param cloud_out 输出点云
 * @param params 输出的车厢顶点
 * @param param_count 车厢顶点个数
 */
extern "C" __declspec(dllexport) void
getTrainVertex(pcl::PointCloud<pcl::PointXYZ> *cloud_in, pcl::PointCloud<pcl::PointXYZ> *cloud_out,
               DetectResult **params,
               int &param_count) {
    try {
        // 输出版本
        std::cout << "Version: " << SCAN_VERSION << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inside(new pcl::PointCloud<pcl::PointXYZ>);
        copyPointCloud(*cloud_in, *cloud_inside);

        auto ts = new TrainScan();
        auto results = ts->detect(cloud_inside, cloud_out);

        param_count = static_cast<int>(results.size());
        *params = new DetectResult[param_count]{};
        for (int i = 0; i < param_count; ++i) {
            (*params)[i] = results[i];
        }

        delete ts;
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}


/**
 * @brief 火车鞍座识别,多车厢
 * @param cloud_in 输入点云
 * @param cloud_out 输出点云
 * @param params 输出的车厢顶点
 * @param param_count 车厢顶点个数
 */
extern "C" __declspec(dllexport) void
getTrainSaddle(pcl::PointCloud<pcl::PointXYZ> *cloud_in, pcl::PointCloud<pcl::PointXYZ> *cloud_out,
               DetectResult **params, int &param_count) {
    try {
        // 输出版本
        std::cout << "Version: " << SCAN_VERSION << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inside(new pcl::PointCloud<pcl::PointXYZ>);
        copyPointCloud(*cloud_in, *cloud_inside);

        auto ts = new TrainScan();
        auto results = ts->detect(cloud_inside, cloud_out);
        

        param_count = static_cast<int>(results.size());
        *params = new DetectResult[param_count];
        for (int i = 0; i < param_count; ++i) {
            (*params)[i] = results[i];
        }

        delete ts;
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}

// filename为点云文件名
// params为输出的ModelParam
// point_n为ModelParam个数
extern "C" __declspec(dllexport) void
getTrainVertexFromCloud(const char *filename, DetectResult **params, int &param_count) {
    try {
        // 输出版本
        std::cout << "Version: " << SCAN_VERSION << std::endl;

        auto ts = new TrainScan();
        auto results = ts->detectFromFile(filename);

        param_count = static_cast<int>(results.size());
        *params = new DetectResult[param_count];
        for (int i = 0; i < param_count; ++i) {
            (*params)[i] = results[i];
        }

        delete ts;
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}

/**
 * @brief 汽车扫描识别
 */
extern "C" __declspec(dllexport) void
getCarVertex(pcl::PointCloud<pcl::PointXYZ> *cloud_in, pcl::PointCloud<pcl::PointXYZ> *cloud_out,
             DetectResult **params, int &param_count) {
    try {
        // 输出版本
        std::cout << "Version: " << SCAN_VERSION << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inside(new pcl::PointCloud<pcl::PointXYZ>);
        copyPointCloud(*cloud_in, *cloud_inside);

        auto carScan = new CarScan();
        carScan->config.loadConfigFromJson("config.json");
        auto results = carScan->detect(cloud_inside, cloud_out);

        param_count = static_cast<int>(results.size());
        *params = new DetectResult[param_count];
        for (int i = 0; i < param_count; ++i) {
            (*params)[i] = results[i];
        }

        delete carScan;
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}


extern "C" __declspec(dllexport) void
getCarVertexTest(pcl::PointCloud<pcl::PointXYZ> *cloud_in, pcl::PointCloud<pcl::PointXYZ> *cloud_out,
                 DetectResult **params, int &param_count) {
    try {
        // 输出版本
        std::cout << "Version: " << SCAN_VERSION << std::endl;

        std::cout << "point cloud size: " << cloud_in->size() << std::endl;
        std::cout << "point cloud width: " << cloud_in->width << std::endl;
        std::cout << "point cloud height: " << cloud_in->height << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inside(new pcl::PointCloud<pcl::PointXYZ>);
        copyPointCloud(*cloud_in, *cloud_inside);

        auto carScan = new CarScan();
        auto path = R"(D:\Work\Chairman\Code\Project\Auto\Coil\Server\scanner\Scan.Algorithm.Test\bin\Debug\net6.0\Data\TestFile\01-CarLocation\20231206103727838.pcd)";
        auto results = carScan->detectFromFile(path);

        std::cout << "result size: " << results.size() << std::endl;

        param_count = static_cast<int>(results.size());
        *params = new DetectResult[param_count];
        for (int i = 0; i < param_count; ++i) {
            (*params)[i] = results[i];
        }

        delete carScan;
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}

/**
 * @brief 汽车AGV扫描导出
 */
extern "C" __declspec(dllexport) void
getAgvVertex(pcl::PointCloud<pcl::PointXYZ> *cloud_in, pcl::PointCloud<pcl::PointXYZ> *cloud_out,
             DetectResult **params, int &param_count) {
    try {
        // 输出版本
        std::cout << "Version: " << SCAN_VERSION << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inside(new pcl::PointCloud<pcl::PointXYZ>);
        copyPointCloud(*cloud_in, *cloud_inside);

        auto agvScan = new AgvScan();
        auto results = agvScan->detect(cloud_inside, cloud_out);

        param_count = static_cast<int>(results.size());
        *params = new DetectResult[param_count];
        for (int i = 0; i < param_count; ++i) {
            (*params)[i] = results[i];
        }
        delete agvScan;
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}


extern "C" __declspec(dllexport) void
cropCloudXyz(pcl::PointCloud<pcl::PointXYZ> *cloud_in, pcl::PointCloud<pcl::PointXYZ> *cloud_out, float min_x,
             float min_y, float min_z, float max_x, float max_y, float max_z) {
    try {
        // 输出版本
        std::cout << "Version: " << SCAN_VERSION << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inside(new pcl::PointCloud<pcl::PointXYZ>);
        copyPointCloud(*cloud_in, *cloud_inside);

        auto outCloud = cloud::cropCloud(cloud_inside, min_x, max_x, min_y, max_y, min_z, max_z);
        copyPointCloud(*outCloud, *cloud_out);
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}
