#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>  
#include <pcl/filters/voxel_grid.h>  
#include <pcl/filters/statistical_outlier_removal.h>  
#include <pcl/filters/conditional_removal.h> 
#include <pcl/filters/radius_outlier_removal.h>  

std::string file_directory;
std::string file_name;
std::string pcd_file;
std::string map_topic_name;
const std::string pcd_format = ".pcd";
nav_msgs::OccupancyGrid map_topic_msg;

double thre_z_min = 0.3;
double thre_z_max = 2.0;
int flag_pass_through = 0;
double grid_x = 0.1;
double grid_y = 0.1;
double grid_z = 0.1;

double map_resolution;
double thre_radius ;
pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud(new pcl::PointCloud<pcl::PointXYZ>);


void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::OccupancyGrid& msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_to_grid");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Rate loop_rate(1.0);
    private_nh.param("file_directory", file_directory, std::string("/home/nubot/T-Hybrid-planner/src/read_pcd/cloud/"));//此处需要修改为自己pcd文件的路径
    ROS_INFO("*** file_directory = %s ***\n", file_directory.c_str());
    private_nh.param("file_name", file_name, std::string("cloud_obs"));
    ROS_INFO("*** file_name = %s ***\n", file_name.c_str());
    pcd_file = file_directory + file_name + pcd_format;
    ROS_INFO("*** pcd_file = %s ***\n", pcd_file.c_str());

    private_nh.param("thre_z_min", thre_z_min, 0.5);
    private_nh.param("thre_z_max", thre_z_max, 2.0);
    private_nh.param("flag_pass_through", flag_pass_through, 0);
    private_nh.param("grid_x", grid_x, 0.1);
    private_nh.param("grid_y", grid_y, 0.1);
    private_nh.param("grid_z", grid_z, 0.1);
    private_nh.param("thre_radius", thre_radius, 0.5);
    private_nh.param("map_resolution", map_resolution, 0.3);
    private_nh.param("map_topic_name", map_topic_name, std::string("map"));

    ros::Publisher map_topic_pub = nh.advertise<nav_msgs::OccupancyGrid>(map_topic_name, 1);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *pcd_cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file: %s \n", pcd_file.c_str());
        return (-1);
    }
    std::cout << "初始点云数据点数：" << pcd_cloud->points.size() << std::endl;
    const clock_t begin_time = clock();
    SetMapTopicMsg(pcd_cloud, map_topic_msg);
    float seconds = float(clock( ) - begin_time) / CLOCKS_PER_SEC;
    std::cout<<seconds<<std::endl;
    while(ros::ok())
    {
        map_topic_pub.publish(map_topic_msg);

        loop_rate.sleep();

        ros::spinOnce();
    }

    return 0;
}


void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::OccupancyGrid& msg)
{
    msg.header.seq = 0;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.info.map_load_time = ros::Time::now();
    msg.info.resolution = map_resolution;
    double x_min, x_max, y_min, y_max;
    if(cloud->points.empty())
    {
        ROS_WARN("pcd is empty!\n");

        return;
    }
    pcl::PointXYZ min_point;
    pcl::PointXYZ max_point;
    pcl::getMinMax3D(*cloud,min_point,max_point);

    x_min=0.0;
    x_max=max_point.x+1;
    y_min=0.0;
    y_max=max_point.y+1;

    msg.info.origin.position.x = 0.0;
    msg.info.origin.position.y = 0.0;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    msg.info.width = int((x_max - x_min) / map_resolution);
    msg.info.height = int((y_max - y_min) / map_resolution);
    msg.data.resize(msg.info.width * msg.info.height);

    ROS_INFO("data size = %d\n", msg.data.size());
   
    for(int iter = 0; iter < cloud->points.size(); iter++)
    {
        int i = int((cloud->points[iter].x - x_min) / map_resolution);
        if(i < 0 || i >= msg.info.width) continue;

        int j = int((cloud->points[iter].y - y_min) / map_resolution);
        if(j < 0 || j >= msg.info.height - 1) continue;
        if(msg.data[i + j * msg.info.width]<100){
          msg.data[i + j * msg.info.width] += 10;
        }    
        else msg.data[i + j * msg.info.width] =100;

    }
// Each occupied grid contains at least two piont clouds
    for(int i=0;i<msg.data.size();++i){
        if(msg.data[i]<20){
            msg.data[i]=0;
        }
        else  msg.data[i]= 100;   
    }

}



