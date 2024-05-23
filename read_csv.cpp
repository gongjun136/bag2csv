#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <queue>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    double timestamp;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(uint16_t, ring, ring))

using PointXYZIRT = VelodynePointXYZIRT;

// 解析 CSV 行
inline std::vector<std::string> parseCSVLine(const std::string &line)
{
    std::vector<std::string> result;
    std::stringstream ss(line);
    std::string item;

    while (std::getline(ss, item, ','))
    {
        result.push_back(item);
    }

    return result;
}

int main(int argc, char const *argv[])
{
    std::ifstream file("../2024-04-29-181006.csv");
    if (!file.is_open())
    {
        std::cerr << "Failed to open file " << std::endl;
        return 0;
    }

    std::string line;
    while (std::getline(file, line))
    {
        auto data = parseCSVLine(line);
        std::string topic = data[0];
        if (topic == "/hesai/pandar")
        {
            pcl::PointCloud<PointXYZIRT> scan;
            scan.clear();
            scan.header.stamp = std::stof(data[1]);

            // 确保点云数据的字段数与每个点所包含的字段数一致
            size_t pointFieldCount = 6; // x, y, z, intensity, timestamp, ring
            size_t baseIndex = 4;       // 跳过前四个公共字段

            size_t pointCount = (data.size() - baseIndex) / pointFieldCount;

            for (size_t i = 0; i < pointCount; ++i)
            {
                size_t index = baseIndex + i * pointFieldCount;
                PointXYZIRT point;
                point.x = std::stof(data[index]);
                point.y = std::stof(data[index + 1]);
                point.z = std::stof(data[index + 2]);
                point.intensity = std::stof(data[index + 3]);
                point.timestamp = std::stod(data[index + 4]);
                point.ring = std::stoi(data[index + 5]);

                scan.push_back(point);
            }

            // 验证读取的数据
            std::cout << "Read " << scan.size() << " points from topic: " << topic << std::endl;
            for (size_t i = 0; i < std::min(scan.size(), size_t(5)); ++i) // 只打印前5个点
            {
                const auto &point = scan[i];
                std::cout << "Point " << i + 1 << ": "
                          << "x=" << point.x << ", "
                          << "y=" << point.y << ", "
                          << "z=" << point.z << ", "
                          << "intensity=" << point.intensity << ", "
                          << "timestamp=" << point.timestamp << ", "
                          << "ring=" << point.ring << std::endl;
            }
        }
    }

    file.close();
    return 0;
}
