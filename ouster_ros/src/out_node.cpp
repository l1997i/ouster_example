#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

static size_t counter = 0;
std::string cloud_mode = "";
std::string out_path = "";
std::string pcd_path = "";
std::string bin_path = "";

pcl::PointCloud<pcl::PointXYZI> msgToPointCloud(const sensor_msgs::PointCloud2ConstPtr& lidar_message) {
    pcl::PointCloud<pcl::PointXYZI> point_cloud;
    pcl::fromROSMsg(*lidar_message, point_cloud);

    return point_cloud;
}

void pcdSubscribePointCloud(const sensor_msgs::PointCloud2ConstPtr& lidar_message) {
    pcl::PointCloud<pcl::PointXYZI> point_cloud = msgToPointCloud(lidar_message);
    counter++;
    
    std::string file_name = pcd_path + std::to_string(counter) + ".pcd";
    ROS_INFO_STREAM(file_name);
    pcl::io::savePCDFile(file_name, point_cloud);
    
}

void binSubscribePointCloud(const sensor_msgs::PointCloud2ConstPtr& lidar_message) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> point_cloud = msgToPointCloud(lidar_message);
    cloud = point_cloud.makeShared();
    counter++;

    std::string file_name = bin_path + std::to_string(counter) + ".bin";
    std::ofstream binFile(file_name.c_str(), std::ios::out | std::ios::binary);
    ROS_INFO_STREAM(file_name);

    for (unsigned int j = 0; j < cloud->size(); j++)
    {
        binFile.write((char *)&cloud->at(j).x, sizeof(cloud->at(j).x));
        binFile.write((char *)&cloud->at(j).y, sizeof(cloud->at(j).y));
        binFile.write((char *)&cloud->at(j).z, sizeof(cloud->at(j).z));
        binFile.write((char *)&cloud->at(j).intensity, sizeof(cloud->at(j).intensity));
    }
    binFile.close();
}

int main(int argc, char **argv) {
    std::string lidar_topic = "/os_cloud_node/points";
    ros::init(argc, argv, "point_cloud_subscriber");
    ros::NodeHandle node_handle;
    cloud_mode = node_handle.param("/out_node/cloud_mode", std::string{});
    out_path = node_handle.param("/out_node/out_path", std::string{});
    mkdir(out_path.c_str(), 0777);

    ros::Subscriber point_cloud_sub;

    if (cloud_mode == "pcd") {
        ROS_INFO_STREAM("Now [PCD] Mode...");
        pcd_path = out_path + "/pcd/";
        mkdir(pcd_path.c_str(), 0777);
        point_cloud_sub = 
            node_handle.subscribe(lidar_topic, 1, pcdSubscribePointCloud);
    }
    else if (cloud_mode == "bin") {
        ROS_INFO_STREAM("Now [BIN] Mode...");
        bin_path = out_path + "/bin/";
        mkdir(bin_path.c_str(), 0777);
        point_cloud_sub = 
            node_handle.subscribe(lidar_topic, 1, binSubscribePointCloud);
    }

    ros::spin();
    return 0;
}
