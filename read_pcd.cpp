#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// 定义点云类型
struct PointXYZITR {
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float intensity;
  double timestamp;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZITR,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (double, timestamp, timestamp)
                                   (uint16_t, ring, ring)
)

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <pcd-file> <output-txt-file>" << std::endl;
        return -1;
    }

    std::string pcd_file = argv[1];
    std::string txt_file = argv[2];

    pcl::PointCloud<PointXYZITR>::Ptr cloud(new pcl::PointCloud<PointXYZITR>);

    if (pcl::io::loadPCDFile<PointXYZITR>(pcd_file, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", pcd_file.c_str());
        return -1;
    }

    std::ofstream outfile(txt_file);
    if (!outfile.is_open()) {
        std::cerr << "Failed to open output file: " << txt_file << std::endl;
        return -1;
    }

    outfile << "Loaded " << cloud->width * cloud->height << " data points from " << pcd_file << std::endl;

    for (const auto& point : *cloud) {
        outfile << "Point (" << point.x << ", " << point.y << ", " << point.z << ") -> "
                << "Intensity: " << point.intensity << ", "
                << "Timestamp: " << point.timestamp << ", "
                << "Ring: " << point.ring << std::endl;
    }

    outfile.close();
    return 0;
}
