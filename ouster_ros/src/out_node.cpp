#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

static size_t counter = 0;
auto cloud_mode = nh.param("cloud_mode", std::string{});
auto out_path = nh.param("out_path", std::string{});

pcl::PointCloud<pcl::PointXYZI> msgToPointCloud(const sensor_msgs::PointCloud2ConstPtr& lidar_message) {
    pcl::PointCloud<pcl::PointXYZI> point_cloud;
    pcl::fromROSMsg(*lidar_message, point_cloud);

    return point_cloud;
}

void pcdSubscribePointCloud(const sensor_msgs::PointCloud2ConstPtr& lidar_message) {
    pcl::PointCloud<pcl::PointXYZI> point_cloud = msgToPointCloud(lidar_message);
    counter++;
    
    std::string file_name = pcd_path + std::to_string(counter) + ".pcd";
    pcl::io::savePCDFile(file_name, point_cloud);
    
}

void binSubscribePointCloud(const sensor_msgs::PointCloud2ConstPtr& lidar_message) {
    pcl::PointCloud<pcl::PointXYZI> point_cloud = msgToPointCloud(lidar_message);
    counter++;

    std::string file_name = bin_path + std::to_string(counter) + ".bin";
    std::ofstream binFile(file_name.c_str(), std::ios::out | std::ios::binary);

    for (int j = 0; j < cloud->size(); j++)
    {
        binFile.write((char *)point_cloud->at(j).x, sizeof(point_cloud->at(j).x));
        binFile.write((char *)point_cloud->at(j).y, sizeof(point_cloud->at(j).y));
        binFile.write((char *)point_cloud->at(j).z, sizeof(point_cloud->at(j).z));
        binFile.write((char *)point_cloud->at(j).intensity, sizeof(point_cloud->at(j).intensity));
    }
    binFile.close();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "point_cloud_subscriber");
    ros::NodeHandle node_handle;
    mkdir(out_path.c_str(), 0777);

    if (cloud_mode = "pcd") {
        std::string pcd_path = out_path + "/pcd/";
        mkdir(pcd_path.c_str(), 0777);
        ros::Subscriber point_cloud_sub = 
            node_handle.subscribe("/os_cloud_node/points", 1, pcdSubscribePointCloud);
    }
    else if (cloud_mode = "bin") {
        std::string bin_path = out_path + "/bin/";
        mkdir(bin_path.c_str(), 0777);
        ros::Subscriber point_cloud_sub = 
            node_handle.subscribe("/os_cloud_node/points", 1, binSubscribePointCloud);
    }

    
    ros::spin();
    return 0;
}

