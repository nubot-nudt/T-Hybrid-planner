#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include<fstream>
#include <pcl/visualization/cloud_viewer.h>



int main(int argc,char **argv){
    ros::init(argc,argv,"read_pcd");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub1=nh.advertise<sensor_msgs::PointCloud2> ("cloud_obs",1);  
    ros::Publisher pcl_pub2=nh.advertise<sensor_msgs::PointCloud2> ("cloud_travel",1);
    ros::Publisher pcl_pub3=nh.advertise<sensor_msgs::PointCloud2> ("cloud_color",1);
    pcl::PointCloud<pcl::PointXYZ> cloud1;
    pcl::PointCloud<pcl::PointXYZ> cloud2;
    pcl::PointCloud<pcl::PointXYZI> cloud3;
    sensor_msgs::PointCloud2 output1;
    sensor_msgs::PointCloud2 output2;
    sensor_msgs::PointCloud2 output3;
    //Change all path to the local pcd file path
    pcl::io::loadPCDFile("/home/nubot/T-Hybrid-planner/src/read_pcd/cloud/cloud_obs.pcd",cloud1);
    pcl::io::loadPCDFile("/home/nubot/T-Hybrid-planner/src/read_pcd/cloud/cloud_travel.pcd",cloud2);
    pcl::io::loadPCDFile("/home/nubot/T-Hybrid-planner/src/read_pcd/cloud/cloud_color.pcd",cloud3);
    //Convert the cloud to ROS message
    pcl::toROSMsg(cloud1,output1);
    //this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer
    pcl::toROSMsg(cloud2,output2);
    output1.header.frame_id="map";
    output2.header.frame_id="map";
    pcl::toROSMsg(cloud3,output3);
    output3.header.frame_id="map";
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub1.publish(output1);
        pcl_pub2.publish(output2);
        pcl_pub3.publish(output3);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
