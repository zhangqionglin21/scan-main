#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "config.h"
#include "extensions.h"

/**
 * @brief 读取配置项，如果不存在则不更新
 * @tparam T 数据类型
 * @param pt  
 * @param key 
 * @param value 
 */
template<typename T>
void tryUpdateValue(const boost::property_tree::ptree &pt, const std::string &key, T &value) {
    boost::optional<T> optValue = pt.get_optional<T>(key);
    if (optValue.has_value()) {
        value = optValue.get();
    }
}


/**
 * @brief 从json文件加载配置
 * @param path 
 */
void ScanConfig::loadConfigFromJson(const std::string &path) {
    // 检查文件是否存在
    std::ifstream file(path);
    if (!file.is_open()) {
        return;
    }
    file.close();

    boost::property_tree::ptree pt;
    boost::property_tree::read_json(path, pt);

    tryUpdateValue(pt, "enable_log", enable_log);
    tryUpdateValue(pt, "enable_debug_pcd", enable_debug_pcd);

    auto optional_car_header_direction = pt.get_optional<std::string>("car_header_direction");
    if (optional_car_header_direction.has_value()) {
        car_header_direction = getDirectionAxis(optional_car_header_direction.get());
    }

    tryUpdateValue(pt, "auto_detect_car_header", auto_detect_car_header);
    tryUpdateValue(pt, "coil_max_width", coil_max_width);
    tryUpdateValue(pt, "coil_max_diameter", coil_max_diameter);
    tryUpdateValue(pt, "cloud_crop_x_min", cloud_crop_x_min);
    tryUpdateValue(pt, "cloud_auto_crop", cloud_auto_crop);
    tryUpdateValue(pt, "cloud_crop_x_max", cloud_crop_x_max);
    tryUpdateValue(pt, "cloud_crop_y_min", cloud_crop_y_min);
    tryUpdateValue(pt, "cloud_crop_y_max", cloud_crop_y_max);
    tryUpdateValue(pt, "cloud_crop_z_min", cloud_crop_z_min);
    tryUpdateValue(pt, "cloud_crop_z_max", cloud_crop_z_max);
    tryUpdateValue(pt, "car_length", car_length);
    tryUpdateValue(pt, "car_width", car_width);
    tryUpdateValue(pt, "car_height", car_height);
    tryUpdateValue(pt, "head_tail_cut_range", head_tail_cut_range);
    tryUpdateValue(pt, "highsided_height_min", highsided_height_min);
    tryUpdateValue(pt, "crop_car_body_offset_y", crop_car_body_offset_y);
    tryUpdateValue(pt, "crop_car_body_offset_x", crop_car_body_offset_x);
    tryUpdateValue(pt, "fence_shrink_distance", fence_shrink_distance);
    tryUpdateValue(pt, "highsided_bottom_z_range", highsided_bottom_z_range);
    tryUpdateValue(pt, "crop_coil_offset_x", crop_coil_offset_x);
    tryUpdateValue(pt, "crop_coil_offset_y", crop_coil_offset_y);
    tryUpdateValue(pt, "crop_coil_offset_z_min", crop_coil_offset_z_min);
    tryUpdateValue(pt, "crop_coil_offset_z_max", crop_coil_offset_z_max);
    tryUpdateValue(pt, "crop_saddle_offset_x", crop_saddle_offset_x);
    tryUpdateValue(pt, "crop_saddle_offset_y", crop_saddle_offset_y);
    tryUpdateValue(pt, "crop_saddle_offset_z_min", crop_saddle_offset_z_min);
    tryUpdateValue(pt, "crop_saddle_offset_z_max", crop_saddle_offset_z_max);
    tryUpdateValue(pt, "saddle_length_min", saddle_length_min);
    tryUpdateValue(pt, "saddle_length_width_ratio_min", saddle_length_width_ratio_min);
    tryUpdateValue(pt, "saddle_length_width_ratio_max", saddle_length_width_ratio_max);
    tryUpdateValue(pt, "saddle_height_min", saddle_height_min);
    tryUpdateValue(pt, "saddle_height_max", saddle_height_max);
    tryUpdateValue(pt, "highsided_saddle_cluster_tolerance", highsided_saddle_cluster_tolerance);
    tryUpdateValue(pt, "saddle_like_rect_min", saddle_like_rect_min);
    tryUpdateValue(pt, "part_saddle_like_rect_min", part_saddle_like_rect_min);
    tryUpdateValue(pt, "part_saddle_width_min_gap", part_saddle_width_min_gap);
    tryUpdateValue(pt, "part_saddle_y_min_gap", part_saddle_y_min_gap);
    tryUpdateValue(pt, "part_saddle_max_gap", part_saddle_max_gap);
    tryUpdateValue(pt, "coil_min_width", coil_min_width);
    tryUpdateValue(pt, "coil_min_diameter", coil_min_diameter);
    tryUpdateValue(pt, "coil_side_crop_length", coil_side_crop_length);
    tryUpdateValue(pt, "coil_axis_crop_threshold", coil_axis_crop_threshold);
    tryUpdateValue(pt, "coil_axis_crop_offset", coil_axis_crop_offset);
    tryUpdateValue(pt, "is_coil_min_length_threshold", is_coil_min_length_threshold);
    tryUpdateValue(pt, "carriage_sor_mean_k", carriage_sor_mean_k);
    tryUpdateValue(pt, "carriage_sor_stddev_mul_thresh", carriage_sor_stddev_mul_thresh);
    tryUpdateValue(pt, "saddles_sor_mean_k", saddles_sor_mean_k);
    tryUpdateValue(pt, "saddles_sor_stddev_mul_thresh", saddles_sor_stddev_mul_thresh);
    tryUpdateValue(pt, "saddle_sor_mean_k", saddle_sor_mean_k);
    tryUpdateValue(pt, "saddle_sor_stddev_mul_thresh", saddle_sor_stddev_mul_thresh);
    tryUpdateValue(pt, "coils_sor_mean_k", coils_sor_mean_k);
    tryUpdateValue(pt, "coils_sor_stddev_mul_thresh", coils_sor_stddev_mul_thresh);
    tryUpdateValue(pt, "coil_sor_mean_k", coil_sor_mean_k);
    tryUpdateValue(pt, "coil_sor_stddev_mul_thresh", coil_sor_stddev_mul_thresh);
    tryUpdateValue(pt, "carriage_cluster_tolerance", carriage_cluster_tolerance);
    tryUpdateValue(pt, "carriage_cluster_min_size", carriage_cluster_min_size);
    tryUpdateValue(pt, "saddle_cluster_tolerance", saddle_cluster_tolerance);
    tryUpdateValue(pt, "saddle_cluster_min_size", saddle_cluster_min_size);
    tryUpdateValue(pt, "coil_cluster_tolerance", coil_cluster_tolerance);
    tryUpdateValue(pt, "coil_cluster_min_size", coil_cluster_min_size);
    tryUpdateValue(pt, "train_inner_crop_offset", train_inner_crop_offset);
    tryUpdateValue(pt, "train_top_border_crop_min_offset_z", train_top_border_crop_min_offset_z);
    tryUpdateValue(pt, "train_top_border_crop_max_offset_z", train_top_border_crop_max_offset_z);
    tryUpdateValue(pt, "min_carriage_interval", min_carriage_interval);
    tryUpdateValue(pt, "min_saddle_interval", min_saddle_interval);
    tryUpdateValue(pt, "min_saddle_coil_interval_x", min_saddle_coil_interval_x);
    tryUpdateValue(pt, "min_saddle_coil_interval_y", min_saddle_coil_interval_y);
    tryUpdateValue(pt, "min_coil_carriage_interval_x", min_coil_carriage_interval_x);
    tryUpdateValue(pt, "min_coil_carriage_interval_y", min_coil_carriage_interval_y);
    tryUpdateValue(pt, "min_saddle_carriage_interval_x", min_saddle_carriage_interval_x);
    tryUpdateValue(pt, "min_saddle_carriage_interval_y", min_saddle_carriage_interval_y);
}

/**
 * @brief 保存配置到json文件
 * @param path 
 */
void ScanConfig::saveConfigToJson(const std::string &path) const {
    boost::property_tree::ptree pt;

    pt.put("enable_log", enable_log);
    pt.put("enable_debug_pcd", enable_debug_pcd);
    pt.put("car_header_direction", getAxisString(car_header_direction));
    pt.put("auto_detect_car_header", auto_detect_car_header);
    pt.put("coil_max_width", coil_max_width);
    pt.put("coil_max_diameter", coil_max_diameter);
    pt.put("cloud_auto_crop", cloud_auto_crop);
    pt.put("cloud_crop_x_min", cloud_crop_x_min);
    pt.put("cloud_crop_x_max", cloud_crop_x_max);
    pt.put("cloud_crop_y_min", cloud_crop_y_min);
    pt.put("cloud_crop_y_max", cloud_crop_y_max);
    pt.put("cloud_crop_z_min", cloud_crop_z_min);
    pt.put("cloud_crop_z_max", cloud_crop_z_max);
    pt.put("car_length", car_length);
    pt.put("car_width", car_width);
    pt.put("car_height", car_height);
    pt.put("head_tail_cut_range", head_tail_cut_range);
    pt.put("highsided_height_min", highsided_height_min);
    pt.put("crop_car_body_offset_y", crop_car_body_offset_y);
    pt.put("crop_car_body_offset_x", crop_car_body_offset_x);
    pt.put("fence_shrink_distance", fence_shrink_distance);
    pt.put("highsided_bottom_z_range", highsided_bottom_z_range);
    pt.put("crop_coil_offset_x", crop_coil_offset_x);
    pt.put("crop_coil_offset_y", crop_coil_offset_y);
    pt.put("crop_coil_offset_z_min", crop_coil_offset_z_min);
    pt.put("crop_coil_offset_z_max", crop_coil_offset_z_max);
    pt.put("crop_saddle_offset_x", crop_saddle_offset_x);
    pt.put("crop_saddle_offset_y", crop_saddle_offset_y);
    pt.put("crop_saddle_offset_z_min", crop_saddle_offset_z_min);
    pt.put("crop_saddle_offset_z_max", crop_saddle_offset_z_max);
    pt.put("saddle_length_min", saddle_length_min);
    pt.put("saddle_length_width_ratio_min", saddle_length_width_ratio_min);
    pt.put("saddle_length_width_ratio_max", saddle_length_width_ratio_max);
    pt.put("saddle_height_min", saddle_height_min);
    pt.put("saddle_height_max", saddle_height_max);
    pt.put("highsided_saddle_cluster_tolerance", highsided_saddle_cluster_tolerance);
    pt.put("saddle_like_rect_min", saddle_like_rect_min);
    pt.put("part_saddle_like_rect_min", part_saddle_like_rect_min);
    pt.put("part_saddle_width_min_gap", part_saddle_width_min_gap);
    pt.put("part_saddle_y_min_gap", part_saddle_y_min_gap);
    pt.put("part_saddle_max_gap", part_saddle_max_gap);
    pt.put("coil_min_width", coil_min_width);
    pt.put("coil_min_diameter", coil_min_diameter);
    pt.put("coil_side_crop_length", coil_side_crop_length);
    pt.put("coil_axis_crop_threshold", coil_axis_crop_threshold);
    pt.put("coil_axis_crop_offset", coil_axis_crop_offset);
    pt.put("is_coil_min_length_threshold", is_coil_min_length_threshold);
    pt.put("carriage_sor_mean_k", carriage_sor_mean_k);
    pt.put("carriage_sor_stddev_mul_thresh", carriage_sor_stddev_mul_thresh);
    pt.put("saddles_sor_mean_k", saddles_sor_mean_k);
    pt.put("saddles_sor_stddev_mul_thresh", saddles_sor_stddev_mul_thresh);
    pt.put("saddle_sor_mean_k", saddle_sor_mean_k);
    pt.put("saddle_sor_stddev_mul_thresh", saddle_sor_stddev_mul_thresh);
    pt.put("coils_sor_mean_k", coils_sor_mean_k);
    pt.put("coils_sor_stddev_mul_thresh", coils_sor_stddev_mul_thresh);
    pt.put("coil_sor_mean_k", coil_sor_mean_k);
    pt.put("coil_sor_stddev_mul_thresh", coil_sor_stddev_mul_thresh);
    pt.put("carriage_cluster_tolerance", carriage_cluster_tolerance);
    pt.put("carriage_cluster_min_size", carriage_cluster_min_size);
    pt.put("saddle_cluster_tolerance", saddle_cluster_tolerance);
    pt.put("saddle_cluster_min_size", saddle_cluster_min_size);
    pt.put("coil_cluster_tolerance", coil_cluster_tolerance);
    pt.put("coil_cluster_min_size", coil_cluster_min_size);
    pt.put("train_inner_crop_offset", train_inner_crop_offset);
    pt.put("train_top_border_crop_min_offset_z", train_top_border_crop_min_offset_z);
    pt.put("train_top_border_crop_max_offset_z", train_top_border_crop_max_offset_z);
    pt.put("min_carriage_interval", min_carriage_interval);
    pt.put("min_saddle_interval", min_saddle_interval);
    pt.put("min_saddle_coil_interval_x", min_saddle_coil_interval_x);
    pt.put("min_saddle_coil_interval_y", min_saddle_coil_interval_y);
    pt.put("min_coil_carriage_interval_x", min_coil_carriage_interval_x);
    pt.put("min_coil_carriage_interval_y", min_coil_carriage_interval_y);
    pt.put("min_saddle_carriage_interval_x", min_saddle_carriage_interval_x);
    pt.put("min_saddle_carriage_interval_y", min_saddle_carriage_interval_y);

    boost::property_tree::write_json(path, pt);
}

Axis ScanConfig::carAxis(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in) const {
    if (auto_detect_car_header) {
        // 获取点云的最大最小值
        auto [min_point, max_point] = cloud::getMinMax(cloud_in);

        // 计算点云的X方向的长度和Y方向的长度
        auto size_x = std::abs(max_point.x - min_point.x);
        auto size_y = std::abs(max_point.y - min_point.y);

        // 如果X方向的长度大于Y方向的长度，则认为X方向是车厢的长度方向，否则认为Y方向是车厢的长度方向
        return size_x > size_y ? X : Y;
    }

    return getAxis(car_header_direction);
}
