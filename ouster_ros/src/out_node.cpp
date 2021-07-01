#include <fstream>
#include <stdio.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

using namespace sensor_msgs;
using namespace message_filters;
using namespace std;


static size_t counter_img01 = 0;
static size_t counter_img02 = 0;
static size_t counter_bin = 0;
ofstream binTS, pcdTS, img01TS, img02TS;
string cloud_mode;
string out_path, pcd_path, bin_path, image_01_path, image_02_path;

pcl::PointCloud<pcl::PointXYZI> msgToPointCloud(const sensor_msgs::PointCloud2ConstPtr& lidar_message) {
    pcl::PointCloud<pcl::PointXYZI> point_cloud;
    pcl::fromROSMsg(*lidar_message, point_cloud);

    return point_cloud;
}

void SubscribeImg(const sensor_msgs::ImageConstPtr &img_msg, int camID) {
    boost::posix_time::ptime ts_posix_time = img_msg->header.stamp.toBoost();
    string ts = boost::posix_time::to_iso_extended_string(ts_posix_time);
    string filename;
    cv_bridge::CvImageConstPtr ptr;

    if (camID == 1) {  // image_01_topic = "/multisense/right/image_rect"
        filename = image_01_path + to_string(counter_img01) + ".png";
        img01TS << ts << endl;
        ptr = cv_bridge::toCvCopy(img_msg, "mono8");
        counter_img01++;
    }
    else if (camID == 2) {  // image_02_topic = "/multisense/left/image_rect_color"
        filename = image_02_path + to_string(counter_img02) + ".png";
        img02TS << ts << endl;
        ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
        counter_img02++;
    }

    cv::Mat img = ptr->image;
    cv::imwrite(filename, img);
}

// void pcdSubscribePointCloud(const sensor_msgs::PointCloud2ConstPtr& lidar_msg) {
//     pcl::PointCloud<pcl::PointXYZI> point_cloud = msgToPointCloud(lidar_msg);
//     counter++;
    
//     string file_name = pcd_path + to_string(counter) + ".pcd";
//     ROS_INFO_STREAM(file_name);
//     pcl::io::savePCDFile(file_name, point_cloud);
    
// }

void binSubscribePointCloud(const sensor_msgs::PointCloud2ConstPtr& lidar_msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> point_cloud = msgToPointCloud(lidar_msg);
    const int32_t PT_SIZE = 4; // sizeof(cloud->at(j).{x .. intensity})
    boost::posix_time::ptime ts_posix_time = lidar_msg->header.stamp.toBoost();
    string ts = boost::posix_time::to_iso_extended_string(ts_posix_time);
    binTS << ts << endl;

    cloud = point_cloud.makeShared();

    string file_name = bin_path + to_string(counter_bin) + ".bin";
    ofstream binFile(file_name.c_str(), ios::out | ios::binary);
    ROS_INFO_STREAM(file_name);

    for (unsigned int j = 0; j < cloud->size(); j++)
    {
        binFile.write((char *)&cloud->at(j).x, PT_SIZE);
        binFile.write((char *)&cloud->at(j).y, PT_SIZE);
        binFile.write((char *)&cloud->at(j).z, PT_SIZE);
        binFile.write((char *)&cloud->at(j).intensity, PT_SIZE);
    }

    counter_bin++;
    binFile.close();
    
}

// void binSubscribePointCloud_C(const sensor_msgs::PointCloud2ConstPtr& lidar_msg) {
//     const int32_t PT_SIZE = 4; // sizeof(cloud->at(j).{x .. intensity})
//     boost::posix_time::ptime ts_posix_time = lidar_msg->header.stamp.toBoost();
//     string ts = boost::posix_time::to_iso_extended_string(ts_posix_time);
//     binTS << ts << endl;
//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
//     pcl::PointCloud<pcl::PointXYZI> point_cloud = msgToPointCloud(lidar_msg);
//     cloud = point_cloud.makeShared();
//     counter++;

//     string file_name = bin_path + to_string(counter) + ".bin";
//     FILE* binFile = fopen(file_name.c_str(), "wb");
//     ROS_INFO_STREAM(file_name);

//     for (unsigned int j = 0; j < cloud->size(); j++)
//     {
//         fwrite((char *)&cloud->at(j).x, 1, PT_SIZE, binFile);
//         fwrite((char *)&cloud->at(j).y, 1, PT_SIZE, binFile);
//         fwrite((char *)&cloud->at(j).z, 1, PT_SIZE, binFile);
//         fwrite((char *)&cloud->at(j).intensity, 1, PT_SIZE, binFile);
//     }
//     fclose(binFile);
// }

void callback(const PointCloud2ConstPtr& lidar_msg, const ImageConstPtr& img_msg01, const ImageConstPtr& img_msg02)
{
    binSubscribePointCloud(lidar_msg);
    SubscribeImg(img_msg01, 1);
    SubscribeImg(img_msg02, 2);
}

int main(int argc, char **argv) {
    const int32_t QUEUE_SIZE = 1000;
    string lidar_topic = "/os_cloud_node/points";
    string image_01_topic = "/multisense/right/image_rect";
    string image_02_topic = "/multisense/left/image_rect_color";
    ros::init(argc, argv, "point_cloud_subscriber");
    ros::NodeHandle node_handle;
    cloud_mode = node_handle.param("/out_node/cloud_mode", string{});

    string dir_name = to_string(ros::Time::now().toSec());
    string base_path = node_handle.param("/out_node/out_path", string{});
    out_path = base_path + "/" + dir_name;
    image_01_path = out_path + "/image_01/";
    image_02_path = out_path + "/image_02/";
    mkdir(base_path.c_str(), 0777);
    mkdir(out_path.c_str(), 0777);
    mkdir(image_01_path.c_str(), 0777);
    mkdir(image_02_path.c_str(), 0777);

    Subscriber<PointCloud2> point_cloud_sub;
    Subscriber<Image> image_01_sub, image_02_sub;
    typedef sync_policies::ApproximateTime<PointCloud2, Image, Image> imgptsSyncPolicy;

    if (cloud_mode == "pcd") {
        ROS_INFO_STREAM("Now [PCD] Mode...");
        pcd_path = out_path + "/pcd/";
        mkdir(pcd_path.c_str(), 0777);
        pcdTS.open(pcd_path + "timestamp.txt");
    }
    else if (cloud_mode == "bin") {
        ROS_INFO_STREAM("Now [BIN] Mode...");
        bin_path = out_path + "/bin/";
        mkdir(bin_path.c_str(), 0777);
        binTS.open(bin_path + "timestamp.txt");
    }

    point_cloud_sub.subscribe(node_handle, lidar_topic, QUEUE_SIZE);
    image_01_sub.subscribe(node_handle, image_01_topic, QUEUE_SIZE);
    image_02_sub.subscribe(node_handle, image_02_topic, QUEUE_SIZE);

    img01TS.open(image_01_path + "timestamp.txt");
    img02TS.open(image_02_path + "timestamp.txt");

    Synchronizer<imgptsSyncPolicy> sync(imgptsSyncPolicy(QUEUE_SIZE), point_cloud_sub, image_01_sub, image_02_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    ros::spin();
    binTS.close();
    pcdTS.close();
    img01TS.close();
    img02TS.close();
    return 0;
}
