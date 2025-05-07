#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <boost/foreach.hpp>
#include <fstream>
#include <set>
#include <livox_ros_driver/CustomMsg.h>

// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>

// // 定义点云类型
// struct PointXYZITR
// {
//     PCL_ADD_POINT4D; // preferred way of adding a XYZ+padding
//     float intensity;
//     double timestamp;
//     uint16_t ring;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
// } EIGEN_ALIGN16;

// POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZITR,
//                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(uint16_t, ring, ring))

void writeImuData(const std::string &topic, const sensor_msgs::Imu::ConstPtr &msg, std::ofstream &file)
{
    file << topic << "," << msg->header.stamp << ",";
    file << msg->orientation.x << "," << msg->orientation.y << "," << msg->orientation.z << "," << msg->orientation.w << ",";
    file << msg->angular_velocity.x << "," << msg->angular_velocity.y << "," << msg->angular_velocity.z << ",";
    file << msg->linear_acceleration.x << "," << msg->linear_acceleration.y << "," << msg->linear_acceleration.z << "\n";
}

void writeLivoxLidarData(const std::string &topic, const livox_ros_driver::CustomMsg::ConstPtr &msg, std::ofstream &file)
{
    file << topic << "," << msg->header.stamp << "," << msg->point_num << "," << static_cast<int>(msg->lidar_id) << ",";

    for (size_t i = 0; i < msg->points.size(); ++i)
    {
        const auto &p = msg->points[i];
        // std::cout<<static_cast<double>(p.offset_time)<<" ";
        uint64_t timestamp_ns = static_cast<uint64_t>(p.offset_time); // offset_time是ns格式
        std::ostringstream timestamp_ss;
        timestamp_ss << timestamp_ns; // 这里写成ns格式，也可以直接写offset_us，看你需求

        file << p.x << "," << p.y << "," << p.z << ",";
        file << static_cast<int>(p.reflectivity) << ",";  // 用 reflectivity
        file << timestamp_ss.str() << ",";                // offset_time
        file << static_cast<int>(p.line) << ",";
    }
    // std::cout<<std::endl;
    file << "\n";
}


void writePointCloudData(const std::string &topic, const sensor_msgs::PointCloud2::ConstPtr &msg, std::ofstream &file)
{
    // std::cout << "header.stamp " << msg->header.stamp << std::endl;
    // std::cout << "header.stamp.sec " << msg->header.stamp.sec << std::endl;
    // std::cout << "header.stamp.nsec " << msg->header.stamp.nsec << std::endl;
    // std::cout << "header.stamp.toSec() " << msg->header.stamp.toSec() << std::endl;
    // std::cout << std::fixed << "header.stamp.toSec() " << msg->header.stamp.toSec() << std::endl;
    // std::cout << "header.stamp.toNSec()  " << msg->header.stamp.toNSec() << std::endl;
    // uint64_t s= msg->header.stamp.toNSec()/1000000000;
    // std::cout<<"s: "<<s<<std::endl;
    file << topic << "," << msg->header.stamp << "," << msg->width << "," << msg->height << ",";

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");
    sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring(*msg, "ring");
    sensor_msgs::PointCloud2ConstIterator<double> iter_timestamp(*msg, "timestamp");
    // std::set<uint16_t> unique_rings;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_ring, ++iter_timestamp)
    {
        uint64_t timestamp_ns = static_cast<uint64_t>(*iter_timestamp * 1e9); // 将秒转换为纳秒
        uint64_t full_seconds = static_cast<uint64_t>(*iter_timestamp);       // 取整数秒部分
        uint64_t fractional_seconds = timestamp_ns % 1000000000;              // 取纳秒部分
        // bug：纳秒部分可能会出现前导零丢失
        std::ostringstream timestamp_ss;
        timestamp_ss << full_seconds << "." << std::setw(9) << std::setfill('0') << fractional_seconds;

        // uint64_t combined_seconds = full_seconds * 1000000000 + fractional_seconds; // 组合整数秒和纳秒

        file << *iter_x << "," << *iter_y << "," << *iter_z << ",";
        file << *iter_intensity << "," << timestamp_ss.str() << "," << *iter_ring << ",";
        // unique_rings.insert(*iter_ring);
    }
    file << "\n";

    // 写入唯一ring的数量
    // file << "Unique rings count: " << unique_rings.size() << "\n";

    // // 将ROS点云消息转换为PCL点云
    // pcl::PointCloud<PointXYZITR>::Ptr pcl_cloud(new pcl::PointCloud<PointXYZITR>);
    // pcl::fromROSMsg(*msg, *pcl_cloud);

    // for (const auto &point : pcl_cloud->points)
    // {
    //     file << point.x << "," << point.y << "," << point.z << ",";
    //     file << point.intensity << "," << point.timestamp << "," << point.ring << ",";
    // }
    // file << "\n";
}

void writeOdometryData(const std::string &topic, const nav_msgs::Odometry::ConstPtr &msg, std::ofstream &file)
{
    file << topic << "," << msg->header.stamp << ",";
    file << msg->pose.pose.position.x << "," << msg->pose.pose.position.y << "," << msg->pose.pose.position.z << ",";
    file << msg->pose.pose.orientation.x << "," << msg->pose.pose.orientation.y << "," << msg->pose.pose.orientation.z << "," << msg->pose.pose.orientation.w << ",";
    file << msg->twist.twist.linear.x << "," << msg->twist.twist.linear.y << "," << msg->twist.twist.linear.z << ",";
    file << msg->twist.twist.angular.x << "," << msg->twist.twist.angular.y << "," << msg->twist.twist.angular.z << "\n";
}

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        ROS_ERROR("Usage: bag_to_csv <bag_file> <output_csv_file>");
        return 1;
    }

    std::string bag_file = argv[1];
    std::string output_csv_file = argv[2];

    rosbag::Bag bag;
    bag.open(bag_file, rosbag::bagmode::Read);

    std::vector<std::string> topics = {"/livox/imu", "/livox/lidar"};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    std::ofstream file(output_csv_file);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open output file: %s", output_csv_file.c_str());
        return 1;
    }

    ros::Time last_timestamp;
    BOOST_FOREACH (rosbag::MessageInstance const m, view)
    {
        if (m.getTime() <= last_timestamp && !last_timestamp.isZero())
        {
            ROS_INFO("Timestamp out of order: current %lu, last %lu", m.getTime().toNSec(), last_timestamp.toNSec());
        }
        last_timestamp = m.getTime();

        sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
        if (imu_msg != NULL)
        {
            writeImuData(m.getTopic(), imu_msg, file);
            continue;
        }

        sensor_msgs::PointCloud2::ConstPtr pc_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (pc_msg != NULL)
        {
            writePointCloudData(m.getTopic(), pc_msg, file);
            continue;
        }

        livox_ros_driver::CustomMsg::ConstPtr livox_msg = m.instantiate<livox_ros_driver::CustomMsg>();
        if (livox_msg != NULL)
        {
            writeLivoxLidarData(m.getTopic(), livox_msg, file);
            continue;
        }

        nav_msgs::Odometry::ConstPtr odo_msg = m.instantiate<nav_msgs::Odometry>();
        if (odo_msg != NULL)
        {
            writeOdometryData(m.getTopic(), odo_msg, file);
            continue;
        }
    }

    file.close();
    bag.close();
    return 0;
}
