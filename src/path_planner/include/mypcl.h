#ifndef MYPCL_H
#define MYPCL_H
//pcl part
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/centroid.h>
#include <pcl/features/feature.h>
#include <pcl/filters/passthrough.h>

#include <boost/function.hpp>
#include <pcl/filters/crop_box.h>
#include <ctime> //需包含该头文件，或者包含<time.h>
#include <pcl/common/common.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/crop_box.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
namespace HybridAStar {
    namespace MyPcl {

//K邻近点云求法线
//利用协方差矩阵求此点的法线(kd tree)
      static  pcl::PointCloud<pcl::Normal>::Ptr Get_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
            //pcl::PointCloud<pcl::Normal> cloud_normal;
            int K = cloud->points.size();
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
            //创建一个空的kdtree对象，并把它传递给法线估计对象
            //基于给出的输入数据集，kdtree将被建立
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
            //为kdtree添加点云数据
            tree->setInputCloud(cloud);
            //估计法线
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
            ne.setInputCloud(cloud);
            ne.setSearchMethod(tree);
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

//cloud_square  提取机器人整体下边的点云，机器人的朝向角有关
      static inline pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_square(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, double x_in, double y_in, double t_in) {
            const clock_t begin_time = clock();
            double W, L, X_1, X_2, X_3, X_4, Y_1, Y_2, Y_3, Y_4;
            W = HybridAStar::Constants::width;
            L = HybridAStar::Constants::length;
            X_1 = L / 2 * cos(t_in) - W / 2 * sin(t_in) + x_in;
            Y_1 = L / 2 * sin(t_in) + W / 2 * cos(t_in) + y_in;

            X_2 = -L / 2 * cos(t_in) - W / 2 * sin(t_in) + x_in;
            Y_2 = -L / 2 * sin(t_in) + W / 2 * cos(t_in) + y_in;

            X_3 = -L / 2 * cos(t_in) + W / 2 * sin(t_in) + x_in;
            Y_3 = -L / 2 * sin(t_in) - W / 2 * cos(t_in) + y_in;

            X_4 = L / 2 * cos(t_in) + W / 2 * sin(t_in) + x_in;
            Y_4 = L / 2 * sin(t_in) - W / 2 * cos(t_in) + y_in;
              float seconds = float(clock() - begin_time) / CLOCKS_PER_SEC;
              std::cout<<"cloud_square_time is"<<seconds<<std::endl;

            pcl::PointCloud<pcl::PointXYZ>::Ptr boundingbox_ptr(new pcl::PointCloud<pcl::PointXYZ>);//用来存放边界
            boundingbox_ptr->push_back(pcl::PointXYZ(X_1, Y_1, -5.0));
            boundingbox_ptr->push_back(pcl::PointXYZ(X_2, Y_2, -5.0));
            boundingbox_ptr->push_back(pcl::PointXYZ(X_3, Y_3, -5.0));
            boundingbox_ptr->push_back(pcl::PointXYZ(X_4, Y_4, -5.0));

            boundingbox_ptr->push_back(pcl::PointXYZ(X_1, Y_1, 5.0));
            boundingbox_ptr->push_back(pcl::PointXYZ(X_2, Y_2, 5.0));
            boundingbox_ptr->push_back(pcl::PointXYZ(X_3, Y_3, 5.0));
            boundingbox_ptr->push_back(pcl::PointXYZ(X_4, Y_4, 5.0));
            pcl::ConvexHull<pcl::PointXYZ> hull;                  //创建凸包对象
            hull.setInputCloud(boundingbox_ptr);                  //设置输入点云
            hull.setDimension(3);                                 //设置凸包维度
            std::vector<pcl::Vertices> polygons;                  //设置向量，用于保存凸包定点
            //std::vector polygons;
            pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);//该点运用于描述凸包形状
            hull.reconstruct(*surface_hull, polygons);            //计算2D凸包结果

            pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::CropHull<pcl::PointXYZ> bb_filter;               //创建crophull对象
            bb_filter.setDim(3);                                  //设置维度：该维度需要与凸包维度一致
            bb_filter.setInputCloud(cloud_in);                       //设置需要滤波的点云
            bb_filter.setHullIndices(polygons);                   //输入封闭多边形的顶点
            bb_filter.setHullCloud(surface_hull);                 //输入封闭多边形的形状
            bb_filter.filter(*objects);                           //执行CropHull滤波，存出结果在objects
                 float seconds1 = float(clock() - begin_time) / CLOCKS_PER_SEC;
                std::cout<<"Get_normal_time is"<<seconds1-seconds<<std::endl;
            //z filter
            pcl::PassThrough<pcl::PointXYZ> pass;
            pcl::PointXYZ min_point;//用于存放三个轴的最小值
            pcl::PointXYZ max_point;//用于存放三个轴的最大值
            pcl::getMinMax3D(*objects, min_point, max_point);
            pass.setInputCloud(objects);            //设置输入点云
            pass.setFilterFieldName("z");         //设置过滤时所需要点云类型的Z字
            pass.setFilterLimits(min_point.z, min_point.z + 1.5);        //设置在过滤字段的范围
            pass.setFilterLimitsNegative(false);   //设置保留范围内还是过滤掉范围内
            pass.filter(*objects);            //执行滤波，保存过滤结果在cloud_filtered
                            float seconds2 = float(clock() - begin_time) / CLOCKS_PER_SEC;
                std::cout<<"Car_angle_time is"<<seconds2-seconds1<<std::endl;
            return objects;
        }

   //cloud_square  提取机器人整体下边的点云，与机器人的朝向角无关   （1*1m机器人适合用）
      static inline pcl::PointCloud<pcl::PointXYZ>::Ptr fast_square(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, double x_in, double y_in) {
            const clock_t begin_time = clock();
            double W;
            W = HybridAStar::Constants::squarLength;
            pcl::PassThrough<pcl::PointXYZ> pass;
            pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
            //filter x
            pass.setInputCloud(cloud_in);            //设置输入点云
            pass.setFilterFieldName("x");         //设置过滤时所需要点云类型的x字
            pass.setFilterLimits(x_in-W/2, x_in + W/2);        //设置在过滤字段的范围
            pass.setFilterLimitsNegative(false);   //设置保留范围内还是过滤掉范围内
            pass.filter(*objects);            //执行滤波，保存过滤结果在cloud_filtered
            // filter y
            pass.setInputCloud(objects);            //设置输入点云
            pass.setFilterFieldName("y");         //设置过滤时所需要点云类型的x字
            pass.setFilterLimits(y_in-W/2, y_in + W/2);        //设置在过滤字段的范围
            pass.setFilterLimitsNegative(false);   //设置保留范围内还是过滤掉范围内
            pass.filter(*objects);            //执行滤波，保存过滤结果在cloud_filtered
            //filter z
            pcl::PointXYZ min_point;//用于存放三个轴的最小值
            pcl::PointXYZ max_point;//用于存放三个轴的最大值
            pcl::getMinMax3D(*objects, min_point, max_point);
            pass.setInputCloud(objects);            //设置输入点云
            pass.setFilterFieldName("z");         //设置过滤时所需要点云类型的Z字
            pass.setFilterLimits(min_point.z, min_point.z + 1.5);        //设置在过滤字段的范围
            pass.setFilterLimitsNegative(false);   //设置保留范围内还是过滤掉范围内
            pass.filter(*objects);            //执行滤波，保存过滤结果在cloud_filtered
            float seconds2 = float(clock() - begin_time) / CLOCKS_PER_SEC;
            std::cout<<"Car_angle_time is"<<seconds2<<std::endl;
            return objects;
        }
     

//compute  distance  points to plane
       static inline double plane_dis(Eigen::Vector3d plane_n, Eigen::Vector4f centroid, Eigen::Vector3f Point_p) {
            double numerator = plane_n(0) * (centroid(0) - Point_p(0))
                               + plane_n(1) * (centroid(1) - Point_p(1)) + plane_n(2) * (centroid(2) - Point_p(2));

            double plan = sqrt(plane_n(0) * plane_n(0) + plane_n(1) * plane_n(1) + plane_n(2) * plane_n(2));
            double distance = numerator / plan;
//    std::cout<<"pingmian"<<plane_n(0)<<"  "<<plane_n(1)<<"  "<<plane_n(2)<<"qiu fang"<<plane_n(0)*plane_n(0)+plane_n(1)*plane_n(1)+plane_n(2)+plane_n(2)<<std::endl;
//    std::cout<<"juli:"<<distance<<"fenzi:"<<numerator<<"fenmu:"<<plan<<std::endl;
            return distance;
        }

//计算两个平面的夹角
      static inline double plane_angle(Eigen::Vector3d n1) {
            Eigen::Vector3d n2;  //水平面
            n2 << 0.0, 0.0, 1.0;
            double cos = std::abs(n1(0) * n2(0) + n1(1) * n2(1) + n1(2) * n2(2));
            double angle = std::acos(cos);
            return angle;
        }

//取一个vector的第二大值、第二小值
     static  inline std::vector<double> max_min_down(std::vector<double> vec_in, int K) {

            for (int i = 0; i < K; ++i) {

                double max_dis = *max_element(vec_in.begin(), vec_in.end());
                double min_dis = *min_element(vec_in.begin(), vec_in.end());
                vec_in.erase(std::remove(vec_in.begin(), vec_in.end(), min_dis), vec_in.end());
                vec_in.erase(std::remove(vec_in.begin(), vec_in.end(), max_dis), vec_in.end());

            }
            return vec_in;

        }

//删除噪点，0.15m内的圆球内要有三个邻居点
       static inline pcl::PointCloud<pcl::PointXYZ>::Ptr dadius_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
            outrem.setInputCloud(cloud_in);
            outrem.setRadiusSearch(0.15);
            outrem.setMinNeighborsInRadius(3);
            // 应用滤波器
            outrem.filter(*cloud_filtered);
            return cloud_filtered;
        }


//计算车姿态，俯仰角、偏滚角
    static inline Eigen::Vector3d  Car_Angle(Eigen::Vector3d Point_nomal,float t_in){
          std::vector<double> angles;
          Eigen::Vector3d eulerAngle;
          Eigen::Vector3d x_1,y_1,x_2,y_2,z_2,fenzi;
          Eigen::Matrix3d rotation_matrix3d;
          int normal_dir,normal_size;

        //   x_1<<cos(t_in),-sin(t_in),0;
        //   y_1<<sin(t_in),cos(t_in),0;
          x_1<<cos(t_in),sin(t_in),0;
          y_1<<-sin(t_in),cos(t_in),0;
          normal_size=x_1.dot(Point_nomal);  //两个向量相乘
          if(normal_size>0){normal_dir=-1;}
          else{normal_dir=1;}
        //   z_2=Point_nomal*normal_dir;
          z_2=Point_nomal;
          fenzi=y_1.cross(z_2);
        //   std::cout<<"fenzi"<<fenzi<<std::endl;
    //    std::cout<<"方向"<<normal_dir<<std::endl;
          x_2=fenzi/fenzi.norm();
          y_2=z_2.cross(x_2);
          rotation_matrix3d<<x_2,y_2,z_2;    //在投影地形上的姿态
        //   std::cout<<"rotation"<<rotation_matrix3d<<std::endl;
        //   std::cout<<"pianhang jiao"<<t_in<<std::endl;
          eulerAngle=rotation_matrix3d.eulerAngles(2,1,0);  //按照ZYX顺序将旋转矩阵转换成欧拉角
          double a=eulerAngle(1);
          double b=eulerAngle(2);
          if(std::abs(a)>1.57){eulerAngle(1)= a/std::abs(a)*(3.1415-std::abs(a));}
          if(std::abs(b)>1.57){eulerAngle(2)= b/std::abs(b)*(3.1415-std::abs(b));}
        //   std::cout<<"oula jiaodu"<<eulerAngle(1)<<"  "<<eulerAngle(2)<<std::endl;
          
          return normal_dir*eulerAngle;

      }
static inline int Sign(double x)
{
         if(x < 0) return -1;
        else return 1;
}

static inline pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,float x_in,float y_in,float  t_in){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out{new pcl::PointCloud<pcl::PointXYZ>};
    Eigen::Vector4f min_point,max_point;
    Eigen::Vector3f rotation_box;
    Eigen::Vector3f translation;
    double W,L,X_1,Y_1,X_3,Y_3;
    double Z_1=5,Z_3=-5;
     W = HybridAStar::Constants::width;
     L = HybridAStar::Constants::length;
    // X_1=L/2* cos(t_in)-W/2* sin(t_in)+x_in;
    // Y_1=L/2* sin(t_in)+W/2* cos(t_in)+y_in;
    
     X_1=W/2;
    Y_1=L/2;
    
    X_3=-W/2;
    Y_3=-L/2;
    
    max_point<<X_1,Y_1,Z_1,1.0;
    min_point<<X_3,Y_3,Z_3,1.0;

rotation_box<<0.0,0.0,t_in-1.57;
translation<<x_in,y_in,0;
    const clock_t begin_time = clock();
    pcl::CropBox<pcl::PointXYZ> box_filter;
    box_filter.setMin(min_point);
    box_filter.setMax(max_point);
    box_filter.setInputCloud(cloud_in);
    box_filter.setNegative(false);
    box_filter.setRotation(rotation_box);
    box_filter.setTranslation(translation);
    box_filter.filter(*cloud_out);
    
//    box_filter.applyFilter(*cloud_out);
    std::cout<<cloud_out->points.size()<<::endl;

    std::cout<<"yunashi"<<cloud_in->points.size()<<"guolv"<<cloud_out->points.size()<<std::endl;
    float seconds = float(clock() - begin_time) / CLOCKS_PER_SEC;
    std::cout<<"time is"<<seconds<<std::endl;
    return cloud_out;
}

static inline pcl::PointCloud<pcl::PointXYZ>::Ptr  round_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane,double x_in,double y_in){
          const clock_t begin_time = clock();
           pcl::PointXYZ searchPoint;
           searchPoint.x=x_in;
           searchPoint.y=y_in;
           searchPoint.z=0.0;
           pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;    
           kdtree.setInputCloud (cloud_plane);
           int K = 100;
        // 半径 R内近邻搜索方法
  
    std::vector<int> pointIdxNKNSearch(K);      //存储查询点近邻索引
     std::vector<float> pointNKNSquaredDistance(K); //存储近邻点对应距离平方
         kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        //  kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) ;
         std::cout<<pointIdxNKNSearch.size()<<std::endl;
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>);
     	                pcl::copyPointCloud(*cloud_in, pointIdxNKNSearch, *cloud_copy);
                         pcl::PointXYZ min_point;//用于存放三个轴的最小值
                         pcl::PointXYZ max_point;//用于存放三个轴的最大值
                         pcl::getMinMax3D(*cloud_copy,min_point,max_point);
                          pcl::PassThrough<pcl::PointXYZ> pass;
                         pass.setInputCloud (cloud_copy);            //设置输入点云
                         pass.setFilterFieldName ("z");         //设置过滤时所需要点云类型的Z字
                         pass.setFilterLimits (min_point.z, min_point.z+1.5);        //设置在过滤字段的范围
                         pass.setFilterLimitsNegative (false);   //设置保留范围内还是过滤掉范围内
                        pass.filter (*cloud_copy);            //执行滤波，保存过滤结果在cloud_filtered
                float seconds = float(clock() - begin_time) / CLOCKS_PER_SEC;
                std::cout<<"time is"<<seconds<<std::endl;
                         return cloud_copy;
     

    }

static inline pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_project(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out{new pcl::PointCloud<pcl::PointXYZ>};
    //创建一个系数为X=Y=0，Z=1的平面
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = coefficients->values[1] = coefficients->values[3] = 0;
    coefficients->values[2] = 1;
        pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud_in);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_out);
    return cloud_out;
}





    }
}


#endif // MYPCL_H
