#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>

#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/radius_outlier_removal.h>


#include <pcl/filters/passthrough.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr Outlier_rmv(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_in);
    sor.setMeanK(100);
    sor.setStddevMulThresh(2);
    sor.filter(*cloud_out);
    return cloud_out;

}

//Down sampling
pcl::PointCloud<pcl::PointXYZ>::Ptr Down_samp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{

    pcl::VoxelGrid<pcl::PointXYZ> sot;
    sot.setInputCloud(cloud_in);
    sot.setLeafSize(0.2f,0.2f,0.2f);
    sot.filter(*cloud_in);
    return cloud_in;
}

//Outlier filter
pcl::PointCloud<pcl::PointXYZ>::Ptr dadius_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud_in);
    outrem.setRadiusSearch(0.2);
    outrem.setMinNeighborsInRadius (5);
    // 应用滤波器
    outrem.filter (*cloud_filtered);
    return cloud_filtered;
}

//PassThrough filter
pcl::PointCloud<pcl::PointXYZ>::Ptr square_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,double x_min,double x_max,double y_min,double y_max,double z_min, double z_max){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_in);           
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("z");         
    pass.setFilterLimits (z_min, z_max);   
    pass.setFilterLimitsNegative (false);   
    pass.filter (*cloud_filtered);        
    return cloud_filtered;
}






int
main (int argc, char** argv)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile ("/home/nubot/T-Hybrid-planner/src/terrain_analysis/cloud/robooster_mapping.pcd", *cloud);
    // cloud= dadius_filter(cloud);
    // cloud = Down_samp(cloud);   
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_terrain(new pcl::PointCloud<pcl::PointXYZ>());

    //Translation
    Eigen::Matrix4f translation_1;
    translation_1<<1.0,0.0,0.0,-12,
            0.0,1.0,0.0,-7.5,
            0.0,0.0,1.0,0.0,
            0.0,0.0,0.0,1.0;

    //Rotary
    Eigen::Matrix4f translation_2;
    translation_2<<0.906,-0.258,0.0,0.0,
            0.258,0.906,0.0,0.0,
            0.0,0.0,1.0,0.0,
            0.0,0.0,0.0,1.0;

    pcl::transformPointCloud (*cloud, *transformed_cloud, translation_1);
//  pcl::transformPointCloud (*transformed_cloud, *transformed_cloud, translation_2);

    pcl::visualization::PCLVisualizer viewer("cloud Viewer");
    viewer.setBackgroundColor(1.0,0.5,1.0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> box_clouds(transformed_cloud,255,10,0);
    viewer.addPointCloud(transformed_cloud,box_clouds,"box_cloud");
    //cloud_terrain=Outlier_rmv(transformed_cloud,cloud_terrain);
    viewer.addCoordinateSystem(10);
    //Save the converted point cloud
    pcl::io::savePCDFileASCII("/home/nubot/T-Hybrid-planner/src/terrain_analysis/cloud/raw_cloud.pcd",*transformed_cloud);
    viewer.spin();
    return 0;
}