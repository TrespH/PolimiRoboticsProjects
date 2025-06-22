#include <iostream>
#include <fstream>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

bool isUnknownPixel(uchar value) {
    return value == 205;
}

cv::Rect findBounds(const cv::Mat& image) {
    int x_min = image.cols, x_max = 0;
    int y_min = image.rows, y_max = 0;

    for (int y = 0; y < image.rows; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            if (!isUnknownPixel(image.at<uchar>(y, x))) {
                x_min = std::min(x_min, x);
                x_max = std::max(x_max, x);
                y_min = std::min(y_min, y);
                y_max = std::max(y_max, y);
            }
        }
    }

    return cv::Rect(x_min, y_min, x_max - x_min + 1, y_max - y_min + 1);
}

std::vector<double> computeCroppedOrigin(
    const cv::Mat& original_img,
    const cv::Rect& bounds,
    double resolution,
    const std::vector<double>& origin)
{
    double ox = origin[0], oy = origin[1], oth = origin[2];

    double dx = bounds.x * resolution;
    double dy = (original_img.rows - bounds.y - bounds.height) * resolution;

    double new_ox = ox + dx * cos(oth) - dy * sin(oth);
    double new_oy = oy + dx * sin(oth) + dy * cos(oth);

    return {new_ox, new_oy, oth};
}

void saveYaml(
    const std::string& filename,
    const std::string& image_file,
    double resolution,
    const std::vector<double>& origin)
{
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "image" << YAML::Value << image_file;
    out << YAML::Key << "resolution" << YAML::Value << resolution;
    out << YAML::Key << "origin" << YAML::Value << origin;
    out << YAML::Key << "negate" << YAML::Value << 0;
    out << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
    out << YAML::Key << "free_thresh" << YAML::Value << 0.196;
    out << YAML::EndMap;

    std::ofstream fout(filename);
    fout << out.c_str();
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " map.yaml [output_basename]\n";
        return 1;
    }

    std::string input_yaml = argv[1];
    std::string base_name = (argc >= 3) ? argv[2] : "cropped";

    YAML::Node map_data = YAML::LoadFile(input_yaml);
    std::string image_file = map_data["image"].as<std::string>();
    double resolution = map_data["resolution"].as<double>();
    std::vector<double> origin = map_data["origin"].as<std::vector<double>>();

    cv::Mat image = cv::imread(image_file, cv::IMREAD_UNCHANGED);
    if (image.empty()) {
        std::cerr << "Cannot read image: " << image_file << "\n";
        return 1;
    }

    cv::Rect bounds = findBounds(image);
    cv::Mat cropped = image(bounds);

    std::string out_image = base_name + ".pgm";
    std::string out_yaml = base_name + ".yaml";

    cv::imwrite(out_image, cropped);

    std::vector<double> new_origin = computeCroppedOrigin(image, bounds, resolution, origin);
    saveYaml(out_yaml, out_image, resolution, new_origin);

    std::cout << "Saved cropped map to: " << out_image << " and " << out_yaml << "\n";
    return 0;
}