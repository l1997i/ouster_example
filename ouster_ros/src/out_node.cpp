#include <fstream>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

static size_t counter = 0;
std::ofstream binTS, pcdTS, img01TS, img02TS;
std::string cloud_mode;
std::string out_path, pcd_path, bin_path, image_01_path, image_02_path;

pcl::PointCloud<pcl::PointXYZI> msgToPointCloud(const sensor_msgs::PointCloud2ConstPtr& lidar_message) {
    pcl::PointCloud<pcl::PointXYZI> point_cloud;
    pcl::fromROSMsg(*lidar_message, point_cloud);

    return point_cloud;
}

void SubscribeImg(const sensor_msgs::ImageConstPtr &img_msg, int camID) {
    boost::posix_time::ptime ts_posix_time = img_msg->header.stamp.toBoost();
    std::string ts = boost::posix_time::to_iso_extended_string(ts_posix_time);
    std::string filename;

    if (camID == 1) {  // image_01_topic = "/multisense/right/image_rect"
        filename = image_01_path + std::to_string(counter) + ".png";
        img01TS << ts << std::endl;
    }
    else if (camID == 2) {  // image_02_topic = "/multisense/left/image_rect_color"
        filename = image_02_path + std::to_string(counter) + ".png";
        img02TS << ts << std::endl;
    }

    cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
    cv::Mat img = ptr->image;
    cv::imwrite(filename, img);
}

void SubscribeImg01(const sensor_msgs::ImageConstPtr &img_msg) {
    SubscribeImg(img_msg, 1);
}

void SubscribeImg02(const sensor_msgs::ImageConstPtr &img_msg) {
    SubscribeImg(img_msg, 2);
}

void pcdSubscribePointCloud(const sensor_msgs::PointCloud2ConstPtr& lidar_msg) {
    pcl::PointCloud<pcl::PointXYZI> point_cloud = msgToPointCloud(lidar_msg);
    counter++;
    
    std::string file_name = pcd_path + std::to_string(counter) + ".pcd";
    ROS_INFO_STREAM(file_name);
    pcl::io::savePCDFile(file_name, point_cloud);
    
}

void binSubscribePointCloud(const sensor_msgs::PointCloud2ConstPtr& lidar_msg) {
    boost::posix_time::ptime ts_posix_time = lidar_msg->header.stamp.toBoost();
    std::string ts = boost::posix_time::to_iso_extended_string(ts_posix_time);
    binTS << ts << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> point_cloud = msgToPointCloud(lidar_msg);
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
    const int32_t QUEUE_SIZE = 1000;
    std::string lidar_topic = "/os_cloud_node/points";
    std::string image_01_topic = "/multisense/right/image_rect";
    std::string image_02_topic = "/multisense/left/image_rect_color";
    ros::init(argc, argv, "point_cloud_subscriber");
    ros::NodeHandle node_handle;
    cloud_mode = node_handle.param("/out_node/cloud_mode", std::string{});
    out_path = node_handle.param("/out_node/out_path", std::string{});
    image_01_path = out_path + "/image_01/";
    image_02_path = out_path + "/image_02/";

    mkdir(out_path.c_str(), 0777);
    mkdir(image_01_path.c_str(), 0777);
    mkdir(image_02_path.c_str(), 0777);

    ros::Subscriber point_cloud_sub;
    ros::Subscriber image_01_sub = node_handle.subscribe(image_01_topic, QUEUE_SIZE, SubscribeImg01);
    ros::Subscriber image_02_sub = node_handle.subscribe(image_02_topic, QUEUE_SIZE, SubscribeImg02);

    if (cloud_mode == "pcd") {
        ROS_INFO_STREAM("Now [PCD] Mode...");
        pcd_path = out_path + "/pcd/";
        mkdir(pcd_path.c_str(), 0777);
        pcdTS.open(pcd_path + "timestamp.txt");
        point_cloud_sub = 
            node_handle.subscribe(lidar_topic, QUEUE_SIZE, pcdSubscribePointCloud);
    }
    else if (cloud_mode == "bin") {
        ROS_INFO_STREAM("Now [BIN] Mode...");
        bin_path = out_path + "/bin/";
        mkdir(bin_path.c_str(), 0777);
        binTS.open(bin_path + "timestamp.txt");
        point_cloud_sub = 
            node_handle.subscribe(lidar_topic, QUEUE_SIZE, binSubscribePointCloud);
    }

    img01TS.open(image_01_path + "timestamp.txt");
    img02TS.open(image_02_path + "timestamp.txt");

    ros::spin();
    binTS.close();
    pcdTS.close();
    img01TS.close();
    img02TS.close();
    return 0;
}
