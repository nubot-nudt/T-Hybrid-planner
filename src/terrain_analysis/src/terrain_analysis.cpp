#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <pcl/common/centroid.h>
#include <pcl/features/feature.h>
#include <pcl/filters/passthrough.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <boost/function.hpp>
#include <pcl/filters/crop_box.h>
#include <ctime> 
#include <pcl/common/common.h>
#include <pcl/filters/radius_outlier_removal.h>
#include<fstream>


//Parameter setting
double volxsize=0.3; //size of small voxel grid
double car_width=0.5, car_length=0.7; //Size of robot
// double vol_length= sqrt(car_length*car_length+car_width*car_width);
double vol_length=0.6;   //Size of the big voxel grid
// vol_length=round(vol_length/volxsize)*volxsize;
int map_length=27;     //map length
int map_width= 15;     //map width



double min_point_z;
// Extract the point cloud in the rectangular boday
pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,Eigen::Vector3f min_point,
                                              Eigen::Vector3f max_point){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::CropBox<pcl::PointXYZ> box_filter;
    std::cout<<min_point(0)<<" "<<min_point(1)<<" "<<min_point(2)<<std::endl;
    box_filter.setMin(Eigen::Vector4f(min_point(0),min_point(1),min_point(2),1.0));
    box_filter.setMax(Eigen::Vector4f(max_point(0),max_point(1),max_point(2),1.0));
    box_filter.setInputCloud(cloud_in);
    box_filter.setNegative(true);
    box_filter.setKeepOrganized(false);

    box_filter.filter(*cloud_out);
    std::cout<<cloud_out->points.size()<<::endl;
    return cloud_out;
}
//PassThrough filter, big grid for edge point clouds 
pcl::PointCloud<pcl::PointXYZ>::Ptr passbox_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,int grid_x,int grid_y,int cellSize){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_in);         
    pass.setFilterFieldName ("x");     
    pass.setFilterLimits (grid_x+0, grid_x+cellSize);      
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_filtered);       

    pass.setInputCloud (cloud_filtered);            
    pass.setFilterFieldName ("y");     
    pass.setFilterLimits (grid_y+0, grid_y+cellSize);       
    pass.setFilterLimitsNegative (false);   
    pass.filter (*cloud_filtered);           

    pcl::PointXYZ min_point;
    pcl::PointXYZ max_point;
    pcl::getMinMax3D(*cloud_filtered,min_point,max_point);
    min_point_z=min_point.z;
    pass.setInputCloud (cloud_filtered);            
    pass.setFilterFieldName ("z");         
    pass.setFilterLimits (min_point.z, min_point.z+1.0);      
    pass.setFilterLimitsNegative (false); 
    pass.filter (*cloud_filtered);           
    return cloud_filtered;
}
//PassThrough filter, big grid
pcl::PointCloud<pcl::PointXYZ>::Ptr big_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,int grid_x,int grid_y,double cellSize,double length){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_in);            
    pass.setFilterFieldName ("x");        

    pass.setFilterLimits (grid_x*cellSize-cellSize, grid_x*cellSize+2*cellSize);       
    pass.setFilterLimitsNegative (false);   
    pass.filter (*cloud_filtered);           

    pass.setInputCloud (cloud_filtered);           
    pass.setFilterFieldName ("y");     

     pass.setFilterLimits (grid_y*cellSize-cellSize, grid_y*cellSize+2*cellSize);        
    pass.setFilterLimitsNegative (false);   
    pass.filter (*cloud_filtered);          

    pcl::PointXYZ min_point;
    pcl::PointXYZ max_point;
    pcl::getMinMax3D(*cloud_filtered,min_point,max_point);
    min_point_z=min_point.z;

    pass.setInputCloud (cloud_filtered);            
    pass.setFilterFieldName ("z");        
    pass.setFilterLimits (min_point.z, min_point.z+1.0);       
    pass.setFilterLimitsNegative (false);   
    pass.filter (*cloud_filtered);     
    return cloud_filtered;
}

//PassThrough filter, small grid
pcl::PointCloud<pcl::PointXYZ>::Ptr small_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,int grid_x,int grid_y,double cellSize){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_in);           
    pass.setFilterFieldName ("x");       
    pass.setFilterLimits (grid_x*cellSize+0.0, grid_x*cellSize+cellSize);    
    pass.setFilterLimitsNegative (false);   
    pass.filter (*cloud_filtered);           

    pass.setInputCloud (cloud_filtered);            
    pass.setFilterFieldName ("y");     
    pass.setFilterLimits (grid_y*cellSize+0.0,grid_y*cellSize+cellSize);  
    pass.setFilterLimitsNegative (false);   
    pass.filter (*cloud_filtered);            
    pass.setInputCloud (cloud_filtered);          
    pass.setFilterFieldName ("z");        
    pass.setFilterLimits (min_point_z, min_point_z+1.0);      
    pass.setFilterLimitsNegative (false);   
    pass.filter (*cloud_filtered);        
    return cloud_filtered;
}

//Extract the point cloud in the rectangular boday
pcl::PointCloud<pcl::PointXYZ>::Ptr square_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,int grid_x,int grid_y,int big_grid_x,int big_grid_y){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_in);            //设置输入点云
    pass.setFilterFieldName ("x");         //设置过滤时所需要点云类型的Z字段
    pass.setFilterLimits (big_grid_x+grid_x*0.25, big_grid_x+0.25+grid_x*0.25);        //设置在过滤字段的范围
    pass.setFilterLimitsNegative (false);   //设置保留范围内还是过滤掉范围内
    pass.filter (*cloud_filtered);            //执行滤波，保存过滤结果在cloud_filtered

    pass.setInputCloud (cloud_filtered);            //设置输入点云
    pass.setFilterFieldName ("y");         //设置过滤时所需要点云类型的Z字段
    pass.setFilterLimits (big_grid_y+grid_y*0.25, big_grid_y+0.25+grid_y*0.25);        //设置在过滤字段的范围
    pass.setFilterLimitsNegative (false);   //设置保留范围内还是过滤掉范围内
    pass.filter (*cloud_filtered);            //执行滤波，保存过滤结果在cloud_filtered
    return cloud_filtered;
}

//Calculate the fitting plane normal vector
pcl::PointCloud<pcl::Normal>::Ptr Get_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    int K=cloud->points.size();
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    //Add point clouds
    tree->setInputCloud(cloud);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);
    ne.setSearchMethod (tree);
    ne.setKSearch(K);  
    ne.compute(*cloud_normals);
    return cloud_normals;
}

//Compute  distance  points to plane
double plane_dis(Eigen::Vector3d plane_n,Eigen::Vector4f centroid,Eigen::Vector3f Point_p)
{
    double numerator=plane_n(0)*(centroid(0)-Point_p(0))
                     +plane_n(1)*(centroid(1)-Point_p(1))+plane_n(2)*(centroid(2)-Point_p(2));

    double  plan=sqrt(plane_n(0)*plane_n(0)+plane_n(1)*plane_n(1)+plane_n(2)*plane_n(2));
    double distance=numerator/plan;
    return distance;
}

//Calculate the Angle between the two planes
double plane_angle(Eigen::Vector3d n1){
    Eigen::Vector3d n2;  //Horizontal plane
    n2<<0.0,0.0,1.0;
    double cos = std::abs(n1(0)* n2(0) + n1(1) * n2(1) + n1(2)* n2(2));
    double angle = std::acos(cos);
    return angle;
}


//Get the Kth largest value in the vector
std::vector<double> max_min_down(std::vector<double> vec_in,int K){

    for(int i=0;i<K;++i){

        double  max_dis =  *max_element(vec_in.begin(),vec_in.end());
        double  min_dis=   *min_element(vec_in.begin(),vec_in.end());
        vec_in.erase(std::remove(vec_in.begin(),vec_in.end(),min_dis),vec_in.end());
        vec_in.erase(std::remove(vec_in.begin(),vec_in.end(),max_dis),vec_in.end());

    }
    return vec_in;
}
//RadiusOutlierRemoval filter
pcl::PointCloud<pcl::PointXYZ>::Ptr dadius_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud_in);
    outrem.setRadiusSearch(0.3);
    outrem.setMinNeighborsInRadius (4);
    outrem.filter (*cloud_filtered);
    return cloud_filtered;
}

//StatisticalOutlierRemoval filter
pcl::PointCloud<pcl::PointXYZ>::Ptr Outlier_rmv(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

    sor.setInputCloud(cloud_in);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.setNegative(true);
    sor.filter(*cloud_filtered);
    return cloud_filtered;
}


int Sign(double x)
{
         if(x < 0) return -1;
        else return 1;
}

int
main ()
{
    int grid_length=(int)floor(map_length/volxsize);
    int grid_width=(int)floor(map_width/volxsize);
    int vol_occupy[grid_width*grid_length]; 
    memset(vol_occupy,0,sizeof(vol_occupy));//fill with 0
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter_child (new pcl::PointCloud<pcl::PointXYZ>);


    pcl::PointCloud<pcl::PointXYZI>::Ptr visilize_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr visilize_cloud_1 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr visilize_cloud_2 (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr visilize_cloud_sum (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZ>  cloud_for_grid;    
    pcl::PointCloud<pcl::PointXYZ>  cloud_for_travel;    
    pcl::PointCloud<pcl::PointXYZ>::Ptr terrain_obs (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr terrain_travel (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("/home/nubot/T-Hybrid-planner/src/terrain_analysis/cloud/raw_cloud.pcd", *cloud);
    pcl::PointCloud<pcl::Normal>::Ptr point_normal(new pcl::PointCloud<pcl::Normal>);


    const clock_t begin_time = clock();
    Eigen::Vector4f centroid;
    Eigen::Vector3d Point_nomal;
    Eigen::Vector3f Point_position;
    std::vector<double> p_dis;
    bool isedge=false;
    int grid_id;


    double dis_value;
    double static_travialValue=0;
    bool traver_flag;
    std::ofstream terrain;
    
    terrain.open("/home/nubot/T-Hybrid-planner/src/data/terrainData.txt");

    Eigen::Vector3d n_normal;
    n_normal<<0.0,0.0,1.0;
    for(int grid_y=0;grid_y<grid_width;++grid_y)
        for(int grid_x=0;grid_x<grid_length;++grid_x){
            traver_flag = true;  
            grid_id=grid_x+grid_y*grid_length;
            dis_value=0; Point_nomal<<0,0,0;
            if(vol_occupy[grid_id]==1){continue;}
            double plan_dis=0;
            double little_plan_dis=0;
            if(grid_x<1/volxsize||grid_y<1/volxsize||grid_x>=(map_length-1)/volxsize||grid_y>=(map_width-1)/volxsize){
                cloud_filter= passbox_filter(cloud, floor(grid_x*volxsize), floor(grid_y*volxsize),1.0);
                isedge= true;
            }
            else{
                cloud_filter= big_plane(cloud,grid_x,grid_y,volxsize,vol_length);
                 isedge= false;
            }
          //Only point.size()>=3,we can cumpute the normal
            if(cloud_filter->points.size()>=3){
                point_normal=Get_normal(cloud_filter);

                pcl::compute3DCentroid(*cloud_filter, centroid);
                Point_nomal(0)=point_normal->points[0].normal_x;
                Point_nomal(1)=point_normal->points[0].normal_y;
                Point_nomal(2)=point_normal->points[0].normal_z;
                double Plane_angle= plane_angle(Point_nomal);
                cloud_filter_child= small_plane(cloud,grid_x,grid_y,volxsize);
                //Normal vector direction correction
                Point_nomal = Sign(n_normal.dot(Point_nomal))*Point_nomal; 
                //Steep slope
                if(Plane_angle>0.3 || Plane_angle<-0.3) {
                      traver_flag = false;  //Impassable areas are stored as 2D map
                      cloud_for_grid += *cloud_filter_child;
                  }
                else{
                    //Gentle slope
                    if(cloud_filter_child->points.size()>0){
                        for(int i=0;i<cloud_filter_child->points.size();++i){
                            Point_position(0)=cloud_filter->points[i].x;
                            Point_position(1)=cloud_filter->points[i].y;
                            Point_position(2)=cloud_filter->points[i].z;
                            //compute distance form plane
                            double vol_dis= plane_dis(Point_nomal,centroid,Point_position);
                           little_plan_dis += std::abs(vol_dis);
                            p_dis.push_back(vol_dis);
                        }
                        p_dis=max_min_down(p_dis,2);
                        double  max_dis =  *max_element(p_dis.begin(),p_dis.end());
                        double  min_dis=   *min_element(p_dis.begin(),p_dis.end());
                        if(std::abs(min_dis)>0.20||std::abs(max_dis)>0.20){
                            traver_flag = false;    //Impassable areas are stored as 2D map
                            cloud_for_grid+=*cloud_filter_child;
                        }
                        else{

                            visilize_cloud_sum->clear();
                            static_travialValue=0.5*std::abs(Plane_angle)/0.3+0.5*little_plan_dis/(cloud_filter_child->points.size());  
                            pcl::copyPointCloud(*cloud_filter_child, *visilize_cloud_sum);
                            if (static_travialValue>1)
                            {
                                static_travialValue=1;
                            }
                  
                             for(int i=0;i<visilize_cloud_sum->points.size();++i){
                                   visilize_cloud_sum->points[i].intensity=100*(1-static_travialValue);
                                  }
                             *visilize_cloud_1+= *visilize_cloud_sum;
                            cloud_for_travel+=*cloud_filter_child;
                            for(int i=0;i<cloud_filter->points.size();++i){
                                Point_position(0)=cloud_filter->points[i].x;
                                Point_position(1)=cloud_filter->points[i].y;
                                Point_position(2)=cloud_filter->points[i].z;
                                //Roughness
                                plan_dis += std::abs(plane_dis(Point_nomal,centroid,Point_position));
                            }
                            dis_value=plan_dis/(cloud_filter->points.size()*0.05);
                        }
                        std::vector <double>().swap(p_dis);

                    }
                    else{   //If there is no point cloud in small grid，only store roughness.
                        for(int i=0;i<cloud_filter->points.size();++i){
                            Point_position(0)=cloud_filter->points[i].x;
                            Point_position(1)=cloud_filter->points[i].y;
                            Point_position(2)=cloud_filter->points[i].z;
                            plan_dis += std::abs(plane_dis(Point_nomal,centroid,Point_position));
                        }

                       dis_value=plan_dis/(cloud_filter->points.size()*0.05);
                    }

                }
                terrain<<grid_x+(int)(grid_y*grid_length)<<" "<<traver_flag<<" "<<dis_value<<" "<<" "<<Point_nomal(0)<<" "<<Point_nomal(1)<<" "<<Point_nomal(2)<<"\n";
            }   //Big grid contains at least 3 point clouds.
            else{
            }
        }

    terrain.close();
    float seconds = float(clock( ) - begin_time) / CLOCKS_PER_SEC;
    std::cout<<seconds<<std::endl;

    *terrain_obs=cloud_for_grid;
    pcl::copyPointCloud(*terrain_obs, *visilize_cloud_2); 
    for(int i=0;i<visilize_cloud_2->points.size();++i){
        visilize_cloud_2->points[i].intensity=0;
    }

    *visilize_cloud=*visilize_cloud_1+*visilize_cloud_2;

    *terrain_travel=cloud_for_travel;

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (255.0, 255.0, 255.0);
    // Set the static terrain traversability 
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(visilize_cloud, "intensity");
    viewer.addPointCloud<pcl::PointXYZI>(visilize_cloud, fildColor, "sample cloud");
    viewer.addCoordinateSystem(2);
    pcl::io::savePCDFileASCII("/home/nubot/T-Hybrid-planner/src/read_pcd/cloud/cloud_obs.pcd",*terrain_obs);
    pcl::io::savePCDFileASCII("/home/nubot/T-Hybrid-planner/src/read_pcd/cloud/cloud_travel.pcd",*terrain_travel);
    pcl::io::savePCDFileASCII("/home/nubot/T-Hybrid-planner/src/read_pcd/cloud/cloud_color.pcd",*visilize_cloud);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return (0);
}



