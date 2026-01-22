#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE TestCarRegcognition

#include <boost/test/included/unit_test.hpp>
#include <boost/filesystem.hpp>
#include <io.h>
#include "scan.h"
#include "train_scan.h"
#include "car_scan.h"
#include "agv_scan.h"
#include "extensions.h"

std::vector<std::string>
list_dir(const char* dir, const char* suffix)
{
    char path[256];
    sprintf(path, "%s%s.%s", dir, "*", suffix);
    std::vector<std::string> files;
    intptr_t hFile = 0;
    struct _finddata_t fileinfo {};
    if ((hFile = _findfirst(path, &fileinfo)) != -1)
    {
        do
        {
            files.push_back(fileinfo.name);
        } while (_findnext(hFile, &fileinfo) == 0);
        _findclose(hFile);
    }
    //sort(files.begin(), files.end());
    return files;
}

#if 0
BOOST_AUTO_TEST_CASE(test_train)
{
    auto scan = std::make_shared<TrainScan>();
    scan->config.enable_debug_pcd = true;

    try {
        auto path = R"(D:\scan-main\dopoints\20250101153750.pcd)";
        auto result = scan->detectFromFile(path);
    }
    catch (std::exception& e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}
#endif

BOOST_AUTO_TEST_CASE(test_car)
{
    try {
        int index = -1;
        std::set<int> used_index;
        while (index != 99)
        {
            std::vector<std::string> pcds = list_dir(R"(D:\scan-main\donghai\)", "pcd");
            for (int i = 0; i < pcds.size(); i++)
            {
                if (i > 0 && pcds[i].substr(0, 8) != pcds[i - 1].substr(0, 8))
                    std::cout << std::endl;
                if (used_index.find(i) != used_index.end())
                {
                    if (i < 10)
                    {
                        std::cout << "[ " << i << "]" << ": " << pcds[i] << std::endl;
                    }
                    else
                    {
                        std::cout << "[" << i << "]" << ": " << pcds[i] << std::endl;
                    }
                }
                else
                {
                    if (i < 10)
                    {
                        std::cout << "  " << i << " " << ": " << pcds[i] << std::endl;
                    }
                    else
                    {
                        std::cout << " " << i << " " << ": " << pcds[i] << std::endl;
                    }
                }
            }
            std::cout << "请输入测试图序号，如需全部测试，请输入999，逐个对照测试输入998: " << std::endl;
            std::cout << "上一张测试图编号为: " << index << std::endl;
            std::cin >> index;
            if (index == 99)
                break;
            if (index != 999 && index != 998 && index > pcds.size())
                continue;
            if (index == 999 || index == 998)
            {
                for (int i = 0; i < pcds.size(); i++)
                {
                    // 清除上次结果
                    std::vector<std::string> temp_pcds = list_dir(R"(D:\scan-main\build\scan_detect\test\Debug\)", "pcd");
                    for (auto& pcd : temp_pcds)
                    {
                        std::string temp_path = "D:/scan-main/build/scan_detect/test/Debug/" + pcd;
                        DeleteFileA(temp_path.c_str());
                    }
                    std::string path = "D:/scan-main/donghai/" + pcds[i];
                    auto scan = std::make_shared<CarScan>();
                    scan->config.loadConfigFromJson("config.json");
                    auto results = scan->detectFromFile(path);
                    std::cout << "结果列表: " << std::endl;
                    int coils_count = 0;
                    int saddles_count = 0;
                    for (auto& result : results)
                    {
                        std::string type;
                        switch (result.data_type)
                        {
                        case DetectType::COIL:
                            coils_count++;
                            type = "[钢卷]";
                            break;
                        case DetectType::SADDLE:
                            saddles_count++;
                            type = "[鞍座]";
                            break;
                        case DetectType::CAR_HEAD_POINT:
                            type = "[车头角点]";
                            break;
                        case DetectType::CAR_TAIL_POINT:
                            type = "[车尾角点]";
                            break;
                        default:
                            type = "未知类型";
                            break;
                        }
                        std::cout << "结果类型: " << type << std::endl;
                        std::cout << "    X, Y, Z: [" << std::to_string(result.x) << ", " << std::to_string(result.y) << ", " << std::to_string(result.z) << "]" << std::endl;
                        std::cout << "    宽度: " << result.width << std::endl;
                        std::cout << "    直径: " << std::to_string(result.diameter) << std::endl;
                        std::cout << "    角度: " << std::to_string(result.axis) << std::endl;
                    }
                    std::cout << i << ": " << pcds[i] << std::endl;
                    if (coils_count + saddles_count != 2)
                    {
                        std::cout << "--------------------------------注意！！！--------------------------------" << std::endl;
                        int flag = -1;
                        while (flag != 1)
                        {
                            std::cout << "输入1继续: " << std::endl;
                            std::cin >> flag;
                            if (std::cin.fail())
                                flag = -1;
                        }
                    }
                    else
                    {
                        std::cout << "识别结果: 鞍座: " << saddles_count << "  钢卷: " << coils_count << std::endl << std::endl;
                        if (index == 998)
                        {
                            int flag = -1;
                            while (flag != 1)
                            {
                                std::cout << "输入1继续: " << std::endl;
                                std::cin >> flag;
                                if (std::cin.fail())
                                    flag = -1;
                            }
                        }
                    }
                }
            }
            else
            {
                // 清除上次结果
                std::vector<std::string> temp_pcds = list_dir(R"(D:\scan-main\bulid\scan_detect\test\Debug\)", "pcd");
                for (auto& pcd : temp_pcds)
                {
                    std::string temp_path = "D:/scan-main/bulid/scan_detect/test/Debug/" + pcd;
                    DeleteFileA(temp_path.c_str());
                }
                used_index.insert(index);
                std::string path = "D:/scan-main/donghai/" + pcds[index];
                auto scan = std::make_shared<CarScan>();
                scan->config.loadConfigFromJson("config.json");
                auto results = scan->detectFromFile(path);
                std::cout << "结果列表: " << std::endl;
                for (auto& result : results)
                {
                    std::string type;
                    switch (result.data_type)
                    {
                    case DetectType::COIL:
                        type = "[钢卷]";
                        break;
                    case DetectType::SADDLE:
                        type = "[鞍座]";
                        break;
                    case DetectType::CAR_HEAD_POINT:
                        type = "[车头角点]";
                        break;
                    case DetectType::CAR_TAIL_POINT:
                        type = "[车尾角点]";
                        break;
                    default:
                        type = "未知类型";
                        break;
                    }
                    std::cout << "结果类型: " << type << std::endl;
                    std::cout << "    X, Y, Z: [" << std::to_string(result.x) << ", " << std::to_string(result.y) << ", " << std::to_string(result.z) << "]" << std::endl;
                    std::cout << "    宽度: " << result.width << std::endl;
                    std::cout << "    直径: " << std::to_string(result.diameter) << std::endl;
                    std::cout << "    角度: " << std::to_string(result.axis) << std::endl;
                }
                std::getchar();
            }
        }
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}

#if 0
BOOST_AUTO_TEST_CASE(test_agv)
{
    auto scan = std::make_shared<AgvScan>();
    scan->config.enable_debug_pcd = true;
    
    auto path = R"(D:\scan-main\donghai\20251014153712862.pcd)";
    std::cout << path << std::endl;
    auto result = scan->detectFromFile(path);
}
#endif

#if 0
BOOST_AUTO_TEST_CASE(test_angle_format)
{
    auto angle = angle::clamp(0);
    BOOST_CHECK_EQUAL(angle, 0);

    angle = angle::clamp(360);
    BOOST_CHECK_EQUAL(angle, 0);

    angle = angle::clamp(361);
    BOOST_CHECK_EQUAL(angle, 1);

    angle = angle::clamp(-1);
    BOOST_CHECK_EQUAL(angle, -1);

    angle = angle::clamp(-361);
    BOOST_CHECK_EQUAL(angle, -1);

    angle = angle::clamp(360 * 1000 + 1);
    BOOST_CHECK_EQUAL(angle, 1);

    angle = angle::clamp(360 * 1000 - 1);
    BOOST_CHECK_EQUAL(angle, -1);
}


/**
 * @brief 测试车厢识别
 * 本测试用例可以测试几个异常场景
 * - 边界裁剪过多，导致库位部分被裁剪，导致库位中心位置偏移
 * - 获取库位左右两侧高度的时候会将相邻的鞍座包含进来，导致判断错误，同样会导致库位中心位置偏移
 */
BOOST_AUTO_TEST_CASE(test_car_saddle_x_position)
{
    auto scan = std::make_shared<CarScan>();
    scan->config.enable_debug_pcd = true;
    try {
        auto path = R"(..\..\..\data\car\20240514090546583-ba.pcd)";
        auto result = scan->detectFromFile(path);

        // 判断结果数量为9
        BOOST_CHECK_EQUAL(result.size(), 9);
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}

/**
 * @brief 测试草垫的情况
 * 本测试用例可以测试几个异常场景
 * - 草垫可正常识别
 */
BOOST_AUTO_TEST_CASE(test_car_saddle_straw)
{
    auto scan = std::make_shared<CarScan>();
    scan->config.enable_debug_pcd = true;
    try {
        auto path = R"(..\..\..\data\car\20240514111009496-ba.pcd)";
        auto result = scan->detectFromFile(path);

        // 判断结果数量为9
        BOOST_CHECK_EQUAL(result.size(), 14);
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}


/**
 * @brief 测试车厢识别
 * 本测试用例可以测试几个异常场景
 * - 第一个库位裁剪后库位没有皮垫导致识别失败
 * - 第一库位识别失败后会导致其他点云也被裁剪，导致所有都识别错误
 */
BOOST_AUTO_TEST_CASE(test_car_saddle_split)
{
    auto scan = std::make_shared<CarScan>();
    scan->config.enable_debug_pcd = true;
    try {
        auto path = R"(..\..\..\data\car\20240516145956882-ba.pcd)";
        auto result = scan->detectFromFile(path);

        // 判断结果数量为9
        BOOST_CHECK_EQUAL(result.size(), 9);
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}

BOOST_AUTO_TEST_CASE(test_car_saddle_base_1)
{
    auto scan = std::make_shared<CarScan>();
    scan->config.enable_debug_pcd = true;
    try {
        auto path = R"(..\..\..\data\car\20240517095831450-ba.pcd)";
        auto result = scan->detectFromFile(path);

        // 判断结果数量为9
        BOOST_CHECK_EQUAL(result.size(), 9);
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}

/**
 * @brief
 */
BOOST_AUTO_TEST_CASE(test_car_saddle_base_2)
{
    auto scan = std::make_shared<CarScan>();
    scan->config.enable_debug_pcd = true;
    try {
        auto path = R"(..\..\..\data\car\20240517142947292-ba.pcd)";
        auto result = scan->detectFromFile(path);

        // 判断结果数量为9
        BOOST_CHECK_EQUAL(result.size(), 9);
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}

/**
 * @brief 测试纯鞍座识别(没有皮垫的鞍座)
 */
BOOST_AUTO_TEST_CASE(test_car_saddle_base_3)
{
    auto scan = std::make_shared<CarScan>();
    scan->config.enable_debug_pcd = true;
    try {
        auto path = R"(..\..\..\data\car\20240520182525034-ba.pcd)";
        auto result = scan->detectFromFile(path);

        // 判断结果数量为8
        BOOST_CHECK_EQUAL(result.size(), 8);
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}


BOOST_AUTO_TEST_CASE(test_car_saddle_base_4)
{
    auto scan = std::make_shared<CarScan>();
    scan->config.enable_debug_pcd = true;
    try {
        auto path = R"(..\..\..\data\car\20240528141002964-ba.pcd)";
        auto result = scan->detectFromFile(path);

        // 判断结果数量为9
        BOOST_CHECK_EQUAL(result.size(), 9);
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}

/**
 * @brief 测试车厢范围的识别，本用例车厢边上有部分护栏，修改前算法识别拟合平面有问题会导致后面裁剪的时候车厢部分被裁剪了，导致识别失败
 * 正常修改后识别应该都正常，而且识别的坐标Y范围在4459左右
 */
BOOST_AUTO_TEST_CASE(test_car_saddle_base_5)
{
    auto scan = std::make_shared<CarScan>();
    scan->config.enable_debug_pcd = true;
    try {
        auto path = R"(..\..\..\data\car\20240604104631629-ba.pcd)";
        auto result = scan->detectFromFile(path);

        // 判断结果数量为11
        BOOST_CHECK_EQUAL(result.size(), 11);
        
        // 所有类型为2的结果的Y应该在4460前后100mm范围内
        for (const auto &item: result) {
            if (item.data_type == 2) {
                BOOST_CHECK(item.y > 4526 - 100);
                BOOST_CHECK(item.y < 4526 + 100);
            }
        }
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}

/**
 * @brief 测试识别的鞍座离钢卷比较近，是否会剔除，以前设置的鞍座中心要求离钢卷边缘至少600mm，但是实际情况有很近的，目前修改为500mm
 * 本测试用例应该输出的结果数量为11=5个钢卷 + 4个鞍座 + 2个车厢范围
 */
BOOST_AUTO_TEST_CASE(test_car_saddle_base_6)
{
    auto scan = std::make_shared<CarScan>();
    scan->config.enable_debug_pcd = true;
    try {
        auto path = R"(..\..\..\data\car\20240528095958188-ba.pcd)";
        auto result = scan->detectFromFile(path);

        // 判断结果数量为11
        BOOST_CHECK_EQUAL(result.size(), 11);
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}


BOOST_AUTO_TEST_CASE(test_car_saddle_base_7)
{
    auto scan = std::make_shared<CarScan>();
    scan->config.enable_debug_pcd = true;
    try {
        auto path = R"(..\..\..\data\car\20240528105659175-ba.pcd)";
        auto result = scan->detectFromFile(path);

        // 判断结果数量为11
        BOOST_CHECK_EQUAL(result.size(), 11);
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}


/**
 * @brief 测试车厢范围的识别，本测试用例因为鞍座比较多而且比较高，导致车厢平面的拟合不准确，从而裁剪获取车厢边界的时候部分点云被裁剪掉了，导致
 * 车厢的边界识别严重错误，修改后车厢边界范围识别的时候不再使用车厢平面裁剪点云，而是使用原始的车厢聚类的点云。
 */
BOOST_AUTO_TEST_CASE(test_car_saddle_base_8)
{
    auto scan = std::make_shared<CarScan>();
    scan->config.enable_debug_pcd = true;
    try {
        auto path = R"(..\..\..\data\car\20240613094828157-ba.pcd)";
        auto result = scan->detectFromFile(path);

        // 判断结果数量为11
        BOOST_CHECK_EQUAL(result.size(), 11);
        
        // 所有鞍座的点云的Y在3968附近
        for (const auto &item: result) {
            if (item.data_type == 2) {
                BOOST_CHECK(item.y > 4010 - 50);
                BOOST_CHECK(item.y < 4010 + 50);
            }
        }
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}


/**
 * @brief 本测试用例为草垫的情况，而且其还不是典型的草垫，高度和宽度都比较大，导致认为是普通的鞍座走的普通鞍座的识别导致识别失败。
 * TODO: 本测试用例的算法还未修改，添加测试用例是为记录这种特殊场景用于后续的算法改进
 */
BOOST_AUTO_TEST_CASE(test_car_saddle_base_9_error)
{
    auto scan = std::make_shared<CarScan>();
    scan->config.enable_debug_pcd = true;
    try {
        auto path = R"(..\..\..\data\car\20240529101715600-ba.pcd)";
        auto result = scan->detectFromFile(path);

        // 判断结果数量为6
        BOOST_CHECK_EQUAL(result.size(), 6);
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}

/**
 * @brief 钢卷裁剪的时候保留的部分鞍座，而这部分鞍座和旁边的连起来的，导致后续部分鞍座识别错误（因为裁剪默认是2m导致部分低点识别错误）
 */
BOOST_AUTO_TEST_CASE(test_car_saddle_base_10)
{
    auto scan = std::make_shared<CarScan>();
    scan->config.enable_debug_pcd = true;
    try {
        auto path = R"(..\..\..\data\car\20240531111928103-ba.pcd)";
        auto result = scan->detectFromFile(path);

        // 判断结果数量为11
        BOOST_CHECK_EQUAL(result.size(), 11);
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}



/**
 * @brief 该场景下部分鞍座的点云数量过少，只有60多个点，导致认为无效的点过滤掉了导致识别库位数量不对，修改后的可以正常识别包含9个鞍座
 * TODO: 目前改场景测试存在问题，待后续优化
 */
BOOST_AUTO_TEST_CASE(test_car_saddle_base_11)
{
    auto scan = std::make_shared<CarScan>();
    scan->config.enable_debug_pcd = true;
    try {
        auto path = R"(..\..\..\data\car\20240624132913943-ba.pcd)";
        auto result = scan->detectFromFile(path);

        // 判断结果数量为11
        BOOST_CHECK_EQUAL(result.size(), 11);
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}


/**
 * @brief 该测试场景下，车厢有往外开的护栏，导致护栏影响车厢边界的识别，同时部分护栏被裁剪到钢卷点云，导致钢卷识别报异常
 * 修改后的能正常识别钢卷，程序不会报异常
 */
BOOST_AUTO_TEST_CASE(test_car_coil_base_1)
{
    auto scan = std::make_shared<CarScan>();
    scan->config.enable_debug_pcd = true;
    try {
        auto path = R"(..\..\..\data\car\20240605144402150-ba.pcd)";
        auto result = scan->detectFromFile(path);

        // 判断结果数量为5
        BOOST_CHECK_EQUAL(result.size(), 5);
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}


/**
 * @brief 该测试场景下，车厢有往外开的护栏，导致护栏影响车厢边界的识别，同时部分护栏被裁剪到钢卷点云，导致钢卷识别报异常
 * 该场景下钢卷只有一个，应该识别出来，且不会识别额外的点云
 */
BOOST_AUTO_TEST_CASE(test_car_coil_base_2)
{
    auto scan = std::make_shared<CarScan>();
    scan->config.enable_debug_pcd = true;
    try {
        auto path = R"(..\..\..\data\car\20240529180740235-ba.pcd)";
        auto result = scan->detectFromFile(path);

        // 判断钢卷的数量为1(data_type == 1的数据)
        int count = 0;
        for (const auto &item: result) {
            if (item.data_type == 1) {
                count++;
            }
        }
        BOOST_CHECK_EQUAL(count, 1);
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}

/**
 * @brief 该测试场景下，多个钢卷紧挨着，导致钢卷点云连在一起出现识别错误的情况。修改后可正常识别，识别的钢卷数量为3个
 */
BOOST_AUTO_TEST_CASE(test_car_coil_base_3)
{
    auto scan = std::make_shared<CarScan>();
    scan->config.enable_debug_pcd = true;
    try {
        auto path = R"(..\..\..\data\car\20240604100241061-ba.pcd)";
        auto result = scan->detectFromFile(path);

        // 判断钢卷的数量为3(data_type == 1的数据)
        int count = 0;
        for (const auto &item: result) {
            if (item.data_type == 1) {
                count++;
            }
        }
        BOOST_CHECK_EQUAL(count, 3);
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}


/**
 * @brief 该测试场景下，多个钢卷紧挨着，导致钢卷点云连在一起出现识别错误的情况。修改后可正常识别，识别的钢卷数量为3个
 */
BOOST_AUTO_TEST_CASE(test_car_coil_base_4)
{
    auto scan = std::make_shared<CarScan>();
    scan->config.enable_debug_pcd = true;
    try {
        auto path = R"(..\..\..\data\car\20240714100621664-ba.pcd)";
        auto result = scan->detectFromFile(path);

        // 判断钢卷的数量为7(data_type == 1的数据)
        int count = 0;
        for (const auto &item: result) {
            if (item.data_type == 1) {
                count++;
            }
        }
        BOOST_CHECK_EQUAL(count, 7);
        
        // 判断总数量为9
        BOOST_CHECK_EQUAL(result.size(), 9);
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}

/**
 * @brief 该场景测试有高低落差的车辆的识别，预期能正常识别鞍座和钢卷，本例中总共7各结果，1个钢卷4个鞍座
 */
BOOST_AUTO_TEST_CASE(test_car_body_base_1)
{
    auto scan = std::make_shared<CarScan>();
    scan->config.enable_debug_pcd = true;
    try {
        auto path = R"(..\..\..\data\car\20240615143250883-ba.pcd)";
        auto result = scan->detectFromFile(path);
        
        // 判断结果数量为7
        BOOST_CHECK_EQUAL(result.size(), 7);

        // 判断钢卷的数量为1(data_type == 1的数据)
        int count = 0;
        for (const auto &item: result) {
            if (item.data_type == 1) {
                count++;
            }
        }
        BOOST_CHECK_EQUAL(count, 1);
        
        // 判断鞍座的数量为4(data_type == 2的数据)
        count = 0;
        for (const auto &item: result) {
            if (item.data_type == 2) {
                count++;
            }
        }
        BOOST_CHECK_EQUAL(count, 4);
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}

BOOST_AUTO_TEST_CASE(test_train_body_base_1)
{
    auto scan = std::make_shared<TrainScan>();
    scan->config.enable_debug_pcd = true;
    try {
        auto path = R"(..\..\..\data\train\20240618144215737-ba.pcd)";
        auto result = scan->detectFromFile(path);

        // 判断车厢数量为12(data_type == 3的数据)
        int count = 0;
        for (const auto &item: result) {
            if (item.data_type == 3) {
                count++;
            }
        }
        BOOST_CHECK_EQUAL(count, 12);
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}

BOOST_AUTO_TEST_CASE(test_avg_coil_base_1)
{
    auto scan = std::make_shared<AgvScan>();
    scan->config.enable_debug_pcd = true;
    try {
        auto path = R"(..\..\..\data\agv\ab_agv_1_coil_format.pcd)";
        auto result = scan->detectFromFile(path);

        // 判断车厢数量为1(data_type == 1的数据)
        int count = 0;
        for (const auto &item: result) {
            if (item.data_type == 1) {
                count++;
            }
        }
        BOOST_CHECK_EQUAL(count, 1);
        
        // 判断鞍座为2(data_type == 2的数据)
        count = 0;
        for (const auto &item: result) {
            if (item.data_type == 2) {
                count++;
            }
        }
        BOOST_CHECK_EQUAL(count, 2);
        
        // 判断钢卷数量为1(data_type == 3的数据)
        count = 0;
        for (const auto &item: result) {
            if (item.data_type == 3) {
                count++;
            }
        }
        BOOST_CHECK_EQUAL(count, 1);
    }
    catch (std::exception &e) {
        std::cout << "异常是: " << e.what() << std::endl;
    }
}

#endif