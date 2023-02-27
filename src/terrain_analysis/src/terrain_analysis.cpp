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
#include <ctime> //需包含该头文件，或者包含<time.h>
#include <pcl/common/common.h>
#include <pcl/filters/radius_outlier_removal.h>
#include<fstream>


//extract too high part for fast planning
//全局变量
double min_point_z;


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
//计算法线(my work)
Eigen::Vector3d Comput_nomal(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_in,
                             pcl::PointXYZ searchPoint,int K){
    Eigen::Vector3d normal;
    //std::vector<int> Indx;
    float nx,ny,nz,curvature;
    //创建KdTreeFLANN对象，并把创建的点云设置为输入,创建一个searchPoint变量作为查询点
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    //设置搜索空间
    kdtree.setInputCloud (Cloud_in);
    std::vector<int> Indx(K);      //存储查询点近邻索引
    std::vector<float> pointNKNSquaredDistance(K); //存储近邻点对应距离平方
    if ( kdtree.nearestKSearch (searchPoint, K, Indx, pointNKNSquaredDistance) > 0 ){
        Eigen::Vector4f xyz_centroid;
        pcl::compute3DCentroid(*Cloud_in,Indx,xyz_centroid);
        Eigen::Matrix3f  covariance_matrix;
        pcl::computeCovarianceMatrix(*Cloud_in,xyz_centroid,covariance_matrix);
        if (Indx.size () < 3 ||
            pcl::computeCovarianceMatrix(*Cloud_in,xyz_centroid,covariance_matrix)== 0)
        {
            nx = ny = nz = curvature = std::numeric_limits<float>::quiet_NaN ();
        }
            // Get the plane normal and surface curvature
        else{
            pcl::solvePlaneParameters(covariance_matrix, nx, ny, nz, curvature);
        }
        normal<<nx,ny,nz;
        return normal;

    }
}
//长方体内点云提取
pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,Eigen::Vector3f min_point,
                                              Eigen::Vector3f max_point){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out{new pcl::PointCloud<pcl::PointXYZ>};
    //定义立体范围
//    pcl::PointIndices::Ptr cloud_filter_indices(new pcl::PointIndices);
//    pcl::PointIndices::Ptr cloud_filter_rest_indices(new pcl::PointIndices);
//    std::vector<int> cloud_filter_ind;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out;

    pcl::CropBox<pcl::PointXYZ> box_filter;
    std::cout<<"yunssss"<<std::endl;
    std::cout<<min_point(0)<<" "<<min_point(1)<<" "<<min_point(2)<<std::endl;
    box_filter.setMin(Eigen::Vector4f(min_point(0),min_point(1),min_point(2),1.0));
    box_filter.setMax(Eigen::Vector4f(max_point(0),max_point(1),max_point(2),1.0));
    box_filter.setInputCloud(cloud_in);
    box_filter.setNegative(true);
    box_filter.setKeepOrganized(false);
//    box_filter.setUserFilterValue(0.1f);
    box_filter.filter(*cloud_out);
    std::cout<<cloud_out->points.size()<<::endl;

    std::cout<<"yunashi"<<cloud_in->points.size()<<"guolv"<<cloud_out->points.size()<<std::endl;
    return cloud_out;
}
//直通滤波提取长方体内点云
pcl::PointCloud<pcl::PointXYZ>::Ptr passbox_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,int grid_x,int grid_y,int volsize){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_in);            //设置输入点云
    pass.setFilterFieldName ("x");         //设置过滤时所需要点云类型的Z字段
    pass.setFilterLimits (grid_x+0, grid_x+volsize);        //设置在过滤字段的范围
    pass.setFilterLimitsNegative (false);   //设置保留范围内还是过滤掉范围内
    pass.filter (*cloud_filtered);            //执行滤波，保存过滤结果在cloud_filtered

    pass.setInputCloud (cloud_filtered);            //设置输入点云
    pass.setFilterFieldName ("y");         //设置过滤时所需要点云类型的Z字段
    pass.setFilterLimits (grid_y+0, grid_y+volsize);        //设置在过滤字段的范围
    pass.setFilterLimitsNegative (false);   //设置保留范围内还是过滤掉范围内
    pass.filter (*cloud_filtered);            //执行滤波，保存过滤结果在cloud_filtered

    pcl::PointXYZ min_point;//用于存放三个轴的最小值
    pcl::PointXYZ max_point;//用于存放三个轴的最大值
    pcl::getMinMax3D(*cloud_filtered,min_point,max_point);
    min_point_z=min_point.z;
    // std::cout<<"最低点z值"<<min_point.z<<std::endl;
    pass.setInputCloud (cloud_filtered);            //设置输入点云
    pass.setFilterFieldName ("z");         //设置过滤时所需要点云类型的Z字
    pass.setFilterLimits (min_point.z, min_point.z+1.0);        //设置在过滤字段的范围
    pass.setFilterLimitsNegative (false);   //设置保留范围内还是过滤掉范围内
    pass.filter (*cloud_filtered);            //执行滤波，保存过滤结果在cloud_filtered
    return cloud_filtered;
}
//直通滤波提取长方体内点云
pcl::PointCloud<pcl::PointXYZ>::Ptr big_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,int grid_x,int grid_y,double volsize,double length){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_in);            //设置输入点云
    pass.setFilterFieldName ("x");         //设置过滤时所需要点云类型的Z字段
    // pass.setFilterLimits (grid_x*volsize+volsize/2-length/2, grid_x*volsize+volsize/2+length/2);        //设置在过滤字段的范围
    pass.setFilterLimits (grid_x*volsize-volsize, grid_x*volsize+2*volsize);        //设置在过滤字段的范围
    pass.setFilterLimitsNegative (false);   //设置保留范围内还是过滤掉范围内
    pass.filter (*cloud_filtered);            //执行滤波，保存过滤结果在cloud_filtered

    pass.setInputCloud (cloud_filtered);            //设置输入点云
    pass.setFilterFieldName ("y");         //设置过滤时所需要点云类型的Z字段
    // pass.setFilterLimits (grid_y*volsize+volsize/2-length/2, grid_y*volsize+volsize/2+length/2);        //设置在过滤字段的范围
     pass.setFilterLimits (grid_y*volsize-volsize, grid_y*volsize+2*volsize);        //设置在过滤字段的范围
    pass.setFilterLimitsNegative (false);   //设置保留范围内还是过滤掉范围内
    pass.filter (*cloud_filtered);            //执行滤波，保存过滤结果在cloud_filtered

    pcl::PointXYZ min_point;//用于存放三个轴的最小值
    pcl::PointXYZ max_point;//用于存放三个轴的最大值
    pcl::getMinMax3D(*cloud_filtered,min_point,max_point);
    min_point_z=min_point.z;
    // std::cout<<"最低点z值"<<min_point.z<<std::endl;
    pass.setInputCloud (cloud_filtered);            //设置输入点云
    pass.setFilterFieldName ("z");         //设置过滤时所需要点云类型的Z字
    pass.setFilterLimits (min_point.z, min_point.z+1.0);        //设置在过滤字段的范围
    pass.setFilterLimitsNegative (false);   //设置保留范围内还是过滤掉范围内
    pass.filter (*cloud_filtered);            //执行滤波，保存过滤结果在cloud_filtered
    return cloud_filtered;
}

//直通滤波提取长方体内点云
pcl::PointCloud<pcl::PointXYZ>::Ptr small_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,int grid_x,int grid_y,double volsize){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_in);            //设置输入点云
    pass.setFilterFieldName ("x");         //设置过滤时所需要点云类型的Z字段
    pass.setFilterLimits (grid_x*volsize+0.0, grid_x*volsize+volsize);        //设置在过滤字段的范围
    pass.setFilterLimitsNegative (false);   //设置保留范围内还是过滤掉范围内
    pass.filter (*cloud_filtered);            //执行滤波，保存过滤结果在cloud_filtered

    pass.setInputCloud (cloud_filtered);            //设置输入点云
    pass.setFilterFieldName ("y");         //设置过滤时所需要点云类型的Z字段
    pass.setFilterLimits (grid_y*volsize+0.0,grid_y*volsize+volsize);        //设置在过滤字段的范围
    pass.setFilterLimitsNegative (false);   //设置保留范围内还是过滤掉范围内
    pass.filter (*cloud_filtered);            //执行滤波，保存过滤结果在cloud_filtered

    // pcl::PointXYZ min_point;//用于存放三个轴的最小值
    // pcl::PointXYZ max_point;//用于存放三个轴的最大值
    // pcl::getMinMax3D(*cloud_filtered,min_point,max_point);
    // min_point_z=min_point.z;
    // std::cout<<"最低点z值"<<min_point.z<<std::endl;
    pass.setInputCloud (cloud_filtered);            //设置输入点云
    pass.setFilterFieldName ("z");         //设置过滤时所需要点云类型的Z字
    pass.setFilterLimits (min_point_z, min_point_z+1.0);        //设置在过滤字段的范围
    pass.setFilterLimitsNegative (false);   //设置保留范围内还是过滤掉范围内
    pass.filter (*cloud_filtered);            //执行滤波，保存过滤结果在cloud_filtered
    return cloud_filtered;
}




//提取正方形内点云
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

//K邻近点云求法线
//利用协方差矩阵求此点的法线(kd tree)
pcl::PointCloud<pcl::Normal>::Ptr Get_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    //pcl::PointCloud<pcl::Normal> cloud_normal;
    int K=cloud->points.size();
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);


    //创建一个空的kdtree对象，并把它传递给法线估计对象
    //基于给出的输入数据集，kdtree将被建立
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    //为kdtree添加点云数据
    tree->setInputCloud(cloud);
    //估计法线
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);
    ne.setSearchMethod (tree);
    //使用半径在查询点周围3厘米范围内的所有邻元素
//    ne.setRadiusSearch(0.1);
    //K邻近搜索
    ne.setKSearch(K);   //how to compute K
    ne.compute(*cloud_normals);

    //将点云数据与法向信息拼接,用于之后的点云规划
//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
//    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    return cloud_normals;
}

//compute  distance  points to plane
double plane_dis(Eigen::Vector3d plane_n,Eigen::Vector4f centroid,Eigen::Vector3f Point_p)
{
    double numerator=plane_n(0)*(centroid(0)-Point_p(0))
                     +plane_n(1)*(centroid(1)-Point_p(1))+plane_n(2)*(centroid(2)-Point_p(2));

    double  plan=sqrt(plane_n(0)*plane_n(0)+plane_n(1)*plane_n(1)+plane_n(2)*plane_n(2));
    double distance=numerator/plan;
//    std::cout<<"pingmian"<<plane_n(0)<<"  "<<plane_n(1)<<"  "<<plane_n(2)<<"qiu fang"<<plane_n(0)*plane_n(0)+plane_n(1)*plane_n(1)+plane_n(2)+plane_n(2)<<std::endl;
//    std::cout<<"juli:"<<distance<<"fenzi:"<<numerator<<"fenmu:"<<plan<<std::endl;
    return distance;
}
//计算两个平面的夹角
double plane_angle(Eigen::Vector3d n1){
    Eigen::Vector3d n2;  //水平面
    n2<<0.0,0.0,1.0;
    double cos = std::abs(n1(0)* n2(0) + n1(1) * n2(1) + n1(2)* n2(2));
    double angle = std::acos(cos);
    return angle;
}


//取一个vector的第二大值、第二小值
std::vector<double> max_min_down(std::vector<double> vec_in,int K){

    for(int i=0;i<K;++i){

        double  max_dis =  *max_element(vec_in.begin(),vec_in.end());
        double  min_dis=   *min_element(vec_in.begin(),vec_in.end());
        vec_in.erase(std::remove(vec_in.begin(),vec_in.end(),min_dis),vec_in.end());
        vec_in.erase(std::remove(vec_in.begin(),vec_in.end(),max_dis),vec_in.end());

    }
    return vec_in;
}
//删除噪点，0.15m内的圆球内要有三个邻居点
pcl::PointCloud<pcl::PointXYZ>::Ptr dadius_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud_in);
    outrem.setRadiusSearch(0.3);
    outrem.setMinNeighborsInRadius (4);
    // 应用滤波器
    outrem.filter (*cloud_filtered);
    return cloud_filtered;
}

//滤波离群点
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
//加载点云

    double volxsize=0.3;
    double car_width=0.5, car_length=0.7;
    // double vol_length= sqrt(car_length*car_length+car_width*car_width);
    double vol_length=0.6;
//    vol_length=round(vol_length/volxsize)*volxsize;
    std::cout<<"vol_length"<<vol_length<<std::endl;
    int cloud_length=27;     //map length
    int cloud_width= 15;     //map width
    int grid_length=(int)floor(cloud_length/volxsize);
    int grid_width=(int)floor(cloud_width/volxsize);
    int vol_occupy[grid_width*grid_length];   //一维数组来存储
    memset(vol_occupy,0,sizeof(vol_occupy));//fill with 0
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter_child (new pcl::PointCloud<pcl::PointXYZ>);


    pcl::PointCloud<pcl::PointXYZI>::Ptr visilize_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr visilize_cloud_1 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr visilize_cloud_2 (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr visilize_cloud_sum (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZ>  cloud_for_grid;    //先不用指针
    pcl::PointCloud<pcl::PointXYZ>  cloud_for_travel;  //先不用指针
    //用于保存最终点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr terrain_obs (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr terrain_travel (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("/home/nubot/T-Hybrid-planner/src/terrain_analysis/cloud/raw_cloud.pcd", *cloud);
    pcl::PointCloud<pcl::Normal>::Ptr point_normal(new pcl::PointCloud<pcl::Normal>);


    const clock_t begin_time = clock();
    // 创建存储点云质心的对象
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
            traver_flag = true;    //每次初始化都是可以通行的，只有遇到坡度过高和粗糙度相对较大时，才不可以通行
            grid_id=grid_x+grid_y*grid_length;
            //每次计算之前先把之前存储的dis_vakue和Point_nomal清空
            dis_value=0; Point_nomal<<0,0,0;
            if(vol_occupy[grid_id]==1){continue;}//占用栅格区域不需要进行评估
                //边缘的体素网格用1*1的栅格计算拟合平面
            double plan_dis=0;
            double little_plan_dis=0;
            if(grid_x<1/volxsize||grid_y<1/volxsize||grid_x>=(cloud_length-1)/volxsize||grid_y>=(cloud_width-1)/volxsize){
                cloud_filter= passbox_filter(cloud, floor(grid_x*volxsize), floor(grid_y*volxsize),1.0);
                isedge= true;
            }
            else{
                //中间的体素网格用车轴半径拟合平面
                cloud_filter= big_plane(cloud,grid_x,grid_y,volxsize,vol_length);
                 isedge= false;
            }
          //只有平面点云不为空，才能进行kd tree计算,only point.size()>=3,we can cumpute the normal
            if(cloud_filter->points.size()>=3){
                point_normal=Get_normal(cloud_filter);

                pcl::compute3DCentroid(*cloud_filter, centroid);
                Point_nomal(0)=point_normal->points[0].normal_x;
                Point_nomal(1)=point_normal->points[0].normal_y;
                Point_nomal(2)=point_normal->points[0].normal_z;
//                std::cout<<cloud_filter->points.size()<<" "<<"平面法向量"<<Point_nomal<<std::endl;
                double Plane_angle= plane_angle(Point_nomal);
                cloud_filter_child= small_plane(cloud,grid_x,grid_y,volxsize);
                //将求得的法向量方向都朝上
                Point_nomal = Sign(n_normal.dot(Point_nomal))*Point_nomal;   //使法向量都朝上


                if(Plane_angle>0.3 || Plane_angle<-0.3) {
                      traver_flag = false;  //绝对可通行性为0，即机器人不管用什么姿态运行都不能够通过此地形，并保存到地形数据,此时存储的flag为false，同时dis_value Point_normal均为0
                      cloud_for_grid += *cloud_filter_child;
                  }
                else{
                    //如果平面坡度较为平缓，需要计算体素网格的粗糙度和倾斜度，用来存储查询表
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
                            traver_flag = false;    //绝对可通行性为0，保存到地形数据中
                            cloud_for_grid+=*cloud_filter_child;
                        }
                        else{

                          //下块代码主要用来可视化可以通行性
                          //计算小体素网格的可通行性：
                            visilize_cloud_sum->clear();
                            //0.5  0.5 的比例
                           static_travialValue=0.5*std::abs(Plane_angle)/0.3+0.5*little_plan_dis/(cloud_filter_child->points.size());  //此静态可通行性是为了能够出一个图显示，并不能最终决定规划时可否通行
                           
                            pcl::copyPointCloud(*cloud_filter_child, *visilize_cloud_sum);
                            if (static_travialValue>1)
                            {
                                static_travialValue=1;
                            }
                  
                             for(int i=0;i<visilize_cloud_sum->points.size();++i){
                                   visilize_cloud_sum->points[i].intensity=100*(1-static_travialValue);
                                // visilize_cloud_sum->points[i].intensity=10*static_travialValue;
                                  }
                             *visilize_cloud_1+= *visilize_cloud_sum;
                            //  std::cout<<"cloud visilize size"<<visilize_cloud_1->points.size()<<"travel value "<<static_travialValue<<std::endl;

                            cloud_for_travel+=*cloud_filter_child;
                            //是可通行栅格，准备记录粗糙度和法向量,Point_normal、cucaodu、podu
                            for(int i=0;i<cloud_filter->points.size();++i){
                                Point_position(0)=cloud_filter->points[i].x;
                                Point_position(1)=cloud_filter->points[i].y;
                                Point_position(2)=cloud_filter->points[i].z;
                                //计算拟合平面粗糙度
                                plan_dis += std::abs(plane_dis(Point_nomal,centroid,Point_position));
                            }
                            dis_value=plan_dis/(cloud_filter->points.size()*0.05);
                        }
                        std::vector <double>().swap(p_dis);

                    }//cloud_child 内有点云
                    else{   //小体素网格内没有点云的话，只计算平面粗糙度
                        for(int i=0;i<cloud_filter->points.size();++i){
                            Point_position(0)=cloud_filter->points[i].x;
                            Point_position(1)=cloud_filter->points[i].y;
                            Point_position(2)=cloud_filter->points[i].z;
                            //计算拟合平面粗糙度
                            plan_dis += std::abs(plane_dis(Point_nomal,centroid,Point_position));
                        }

                       dis_value=plan_dis/(cloud_filter->points.size()*0.05);
                    }

                }
                terrain<<grid_x+(int)(grid_y*grid_length)<<" "<<traver_flag<<" "<<dis_value<<" "<<" "<<Point_nomal(0)<<" "<<Point_nomal(1)<<" "<<Point_nomal(2)<<"\n";
            }   //大栅格内有至少三个点云的
            else{
                // std::cout<<"big_kong"<<grid_x<<" "<<grid_y<<std::endl;
            }
            //  terrain<<grid_x+(int)(grid_y*grid_length)<<" "<<dis_value<<" "<<" "<<Point_nomal(0)<<" "<<Point_nomal(1)<<" "<<Point_nomal(2)<<"\n";
        }

     //输出一下vol_occupy里边的内容
    terrain.close();
    float seconds = float(clock( ) - begin_time) / CLOCKS_PER_SEC;
    std::cout<<seconds<<std::endl;

    *terrain_obs=cloud_for_grid;//将最后提取的不可通行点云赋给指针点云
    pcl::copyPointCloud(*terrain_obs, *visilize_cloud_2); 
 for(int i=0;i<visilize_cloud_2->points.size();++i){
     visilize_cloud_2->points[i].intensity=0;
 }

    *visilize_cloud=*visilize_cloud_1+*visilize_cloud_2;

    *terrain_travel=cloud_for_travel;

    //不删除噪点的话 ，是否还会出现空洞重影
    // terrain_obs= dadius_filter(terrain_obs);  //删除噪点
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (255.0, 255.0, 255.0);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> box_clouds(terrain_obs,255,10,0);
    // viewer.addPointCloud(terrain_obs,box_clouds,"box_cloud");
     pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(visilize_cloud, "intensity"); // 按照z字段进行渲染
    viewer.addPointCloud<pcl::PointXYZI>(visilize_cloud, fildColor, "sample cloud");


    viewer.addCoordinateSystem(2);
    pcl::io::savePCDFileASCII("/home/nubot/T-Hybrid-planner/src/read_pcd/cloud/cloud_obs.pcd",*terrain_obs);
    pcl::io::savePCDFileASCII("/home/nubot/T-Hybrid-planner/src/read_pcd/cloud/cloud_travel.pcd",*terrain_travel);
    pcl::io::savePCDFileASCII("/home/nubot/T-Hybrid-planner/src/read_pcd/cloud/cloud_color.pcd",*visilize_cloud);
//    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud_filter, point_normal,1,0.1,"normal");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return (0);
}



