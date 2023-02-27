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

//extract too high part for fast planning
std::vector<pcl::PointCloud<pcl::PointXYZ>> Extract_h(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,\
                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out1,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out2){
    std::vector<int> h_idx;
    std::vector<int> l_idx;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> filter_cloud;

    for(int i=0;i<cloud_in->points.size();++i){
        if(cloud_in->points[i].z>1){
            h_idx.push_back(i);
        }
        else{
            l_idx.push_back(i);
        }
    }
    pcl::copyPointCloud(*cloud_in,l_idx,*cloud_out1);
    pcl::copyPointCloud(*cloud_in,h_idx,*cloud_out2);
    filter_cloud.push_back(*cloud_out1);
    filter_cloud.push_back(*cloud_out2);
    return filter_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Outlier_rmv(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    std::cout<<"guolv"<<std::endl;
    sor.setInputCloud(cloud_in);
    sor.setMeanK(100);
    sor.setStddevMulThresh(2);
    //sor.setNegative(true);
    sor.filter(*cloud_out);

    return cloud_out;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr Down_samp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{

    pcl::VoxelGrid<pcl::PointXYZ> sot;
    sot.setInputCloud(cloud_in);
    sot.setLeafSize(0.2f,0.2f,0.2f);
    sot.filter(*cloud_in);
    return cloud_in;
}

//靠近距离下采样
pcl::PointCloud<pcl::PointXYZ>::Ptr Distance_Down_samp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in){
    double point_dis;
    pcl::PointXYZ searchPoint;
    //利用K近邻进行数据删除
    //创建KdTreeFLANN对象，并把创建的点云设置为输入,创建一个searchPoint变量作为查询点
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    //设置搜索空间
    kdtree.setInputCloud (cloud_in);
    int K = 20;
    std::vector<int> pointIdxNKNSearch(K);      //存储查询点近邻索引
    std::vector<float> pointNKNSquaredDistance(K); //存储近邻点对应距离平方

    std::cout<<"点云"<<std::endl;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out;
    for(int i=0;i<cloud_in->points.size();++i){
        searchPoint=cloud_in->points[i];
        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
//            std::cout<<"邻近点索引"<<pointIdxNKNSearch[1]<<"xiayige"<<pointIdxNKNSearch[2]<<std::endl;
            for (size_t j = 0; j < pointIdxNKNSearch.size (); ++j){
                pcl::PointCloud<pcl::PointXYZ>::iterator index;
                double dis_x=cloud_in->points[ pointIdxNKNSearch[j] ].x -cloud_in->points[i].x;
                double dis_y=cloud_in->points[ pointIdxNKNSearch[j] ].y -cloud_in->points[i].y;
                double dis_z=cloud_in->points[ pointIdxNKNSearch[j] ].z -cloud_in->points[i].z;
                point_dis= sqrt(dis_x*dis_x+dis_y*dis_y+dis_z*dis_z);
                if(point_dis<0.03){
                    index=cloud_in->begin()+pointIdxNKNSearch[j];
                    cloud_in->erase(index);
                }
            }

        }
        std::cout<<"点云大小为："<<cloud_in->points.size()<<std::endl;
    }
    return cloud_in;
}
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



pcl::PointCloud<pcl::PointXYZ>::Ptr square_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,double x_min,double x_max,double y_min,double y_max,double z_min, double z_max){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_in);           //cloud_in      /执行滤波，保存过滤结果在cloud_filtered


    pass.setInputCloud (cloud_filtered);            //设置输入点云
    pass.setFilterFieldName ("z");         //设置过滤时所需要点云类型的Z字段
    pass.setFilterLimits (z_min, z_max);        //设置在过滤字段的范围
    pass.setFilterLimitsNegative (false);   //设置保留范围内还是过滤掉范围内
    pass.filter (*cloud_filtered);            //执行滤波，保存过滤结果在cloud_filtered
    return cloud_filtered;
}






int
main (int argc, char** argv)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color_1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color_2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color_3 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile ("/home/nubot/T-Hybrid-planner/src/terrain_analysis/cloud/robooster_mapping.pcd", *cloud);



    // cloud= dadius_filter(cloud);
    // cloud = Down_samp(cloud);   //降低采样

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_terrain(new pcl::PointCloud<pcl::PointXYZ>());
//平移(-1.25,4.375,-1.4038)
    Eigen::Matrix4f translation_1;
   translation_1<<1.0,0.0,0.0,-12,
           0.0,1.0,0.0,-7.5,
           0.0,0.0,1.0,0.0,
           0.0,0.0,0.0,1.0;

    // translation_1<<-1.0,0.0,0.0,0.0,
    //        0.0,-1.0,0.0,0.0,
    //        0.0,0.0,1.0,0.0,
    //        0.0,0.0,0.0,1.0;
    

//    pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();
//    for(int i=0;i<cloud->points.size();++i){
////        if(cloud->points[i].x>20 &&cloud->points[i].x<34.4 &&cloud->points[i].y>6 &&cloud->points[i].y<19 && cloud->points[i].z>-0.7){
//        if(cloud->points[i].x>0 &&cloud->points[i].x<22.2 &&cloud->points[i].y>0 &&cloud->points[i].y<18.2 && cloud->points[i].z>-0.7){
//            cloud->points[i].z=-0.8;
////cloud->erase(index+i);
//        }
//    }
//
//

////pingyi
//    Eigen::Matrix4f translation_2;

//    translation_2<<1.0,0.0,0.0,13.0,
//             0.0,1.0,0.0,18,
//             0.0,0.0,1.0,0.0,
//             0.0,0.0,0.0,1.0;


////平移(-1.25,4.375,-1.4038)
//    Eigen::Matrix4f translation_1;
//    translation_1<<1.0,0.0,0.0,-51.108,
//            0.0,1.0,0.0,-604.934,
//            0.0,0.0,1.0,-445.563,
//            0.0,0.0,0.0,1.0;
//
//
////旋转转换15du  y
   Eigen::Matrix4f translation_2;

   translation_2<<0.906,-0.258,0.0,0.0,
           0.258,0.906,0.0,0.0,
           0.0,0.0,1.0,0.0,
           0.0,0.0,0.0,1.0;
//
   //旋转转换-90du  z
//    Eigen::Matrix4f translation_3;

//    translation_3<<0.9994,-0.035,0.0,0.0,
//           0.035,0.9994,0.0,0.0,
//            0.0,0.0,1.0,0.0,
//            0.0,0.0,0.0,1.0;
//
//    //y   5
//    Eigen::Matrix4f translation_4;
//
//    translation_4<<0.9659,0.0,0.2588,0.0,
//            0.0,1.0,0.0,0.0,
//            -0.2588,0.0,0.9659,0.0,
//            0.0,0.0,0.0,1.0;
//
    pcl::transformPointCloud (*cloud, *transformed_cloud, translation_1);
//    pcl::transformPointCloud (*transformed_cloud, *transformed_cloud, translation_2);



  //遍历点云
//   for(int i = 0; i < transformed_cloud->points.size(); i++){
//      if(transformed_cloud->points[i].x >= 14.4 && transformed_cloud->points[i].x <= 17.2 && transformed_cloud->points[i].y >= 10.3 && transformed_cloud->points[i].y <= 12.3){
//          transformed_cloud->points[i].z = -0.01;
//      }
//   }





//    pcl::transformPointCloud (*transformed_cloud, *transformed_cloud, translation_3);
//
//    pcl::transformPointCloud (*transformed_cloud, *transformed_cloud, translation_4);

    pcl::visualization::PCLVisualizer viewer("cloud Viewer");

    viewer.setBackgroundColor(1.0,0.5,1.0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> box_clouds(transformed_cloud,255,10,0);
    viewer.addPointCloud(transformed_cloud,box_clouds,"box_cloud");
    //cloud_terrain=Outlier_rmv(transformed_cloud,cloud_terrain);
//   pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//   viewer.addPointCloud<pcl::PointXYZRGB> (cloud_color_3, rgb, "sample cloud");

    viewer.addCoordinateSystem(10);
    pcl::io::savePCDFileASCII("/home/nubot/T-Hybrid-planner/src/terrain_analysis/cloud/raw_cloud.pcd",*transformed_cloud);
    viewer.spin();

    return 0;
}