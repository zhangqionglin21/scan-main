【使用教程】
1、解决方案地址：scan-main\build\scan.sln，快速测试时请解压至F:\scan-main文件夹中。
2、解决方案scan，包含了cylinder_fitting、LMS3DSCAN、scan_bin、scan_detect及test_scan。修改算法后Release模式重新生成scan_detect即可，修改测试方法后Debug模式重新生成test_scan即可，会自动编译需要调用的项目。不要重新生成整个解决方案，LMS3DSCAN（实际未使用）会报错。
3、重新生成的动态库地址：scan-main\bulid\scan_detect\src\Release\scan_detect.dll
4、测试程序地址：scan-main\bulid\scan_detect\test\Debug\test_scan.exe
5、动态库配置文件参考：scan-main\bulid\scan_detect\test\Debug\config.example.json，配置文件支持热修改，每次调用函数时均会重新加载。

【部分代码位置介绍】
1、供调用的函数定义：scan-main\scan_detect\src\scan_export.h，实现：scan-main\scan_detect\src\scan_export.cpp，其中的extern "C" __declspec(dllexport) void 
getCarVertex(pcl::PointCloud<pcl::PointXYZ> *cloud_in, pcl::PointCloud<pcl::PointXYZ> *cloud_out, DetectResult **params, int &param_count)
2、CarScan为Scan类的子类，部分方法进行了重载，修改代码时请注意（Visual Studio跳转有时会跳错）。
3、PCL 1.11.1\3rdParty\FLANN\include\flann\util\params.h中的报错可以不处理，不影响编译成功与实际使用。
4、测试方法：scan-main\scan_detect\test\test_demo.cpp，需要进行其他测试请根据BOOST_AUTO_TEST_CASE(test_car)修改并将其注释掉。
5、正式部署的config.json中enable_debug_pcd一定要为false，否则会保存处理过程中用于debug的点云。