#include "planner.h"

using namespace HybridAStar;
//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner() {
  // _____
  // TODOS
  //    initializeLookups();
  // Lookup::collisionLookup(collisionLookup);
  // ___________________
  // COLLISION DETECTION
  //    CollisionDetection configurationSpace;
  // _________________
  // TOPICS TO PUBLISH
  pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);
  pubGoalpose=n.advertise<geometry_msgs::PoseStamped>("/goal_pose",10);

  // ___________________
  // TOPICS TO SUBSCRIBE
  if (Constants::manual) {
    subMap = n.subscribe("/map", 1, &Planner::setMap, this);
  } else {
    subMap = n.subscribe("/occ_map", 1, &Planner::setMap, this);
  }

  subGoal = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this);
  subStart = n.subscribe("/initialpose", 1, &Planner::setStart, this);
  // subStart = n.subscribe("/odom", 1, &Planner::setStart, this);

  // subrePlanmsg = n.subscribe("/update_plan", 1, &Planner::updatePath, this);
};

//###################################################
//                                       LOOKUPTABLES
//###################################################

void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }

  Lookup::collisionLookup(collisionLookup);
}




//###################################################
//       MAP   首先是加载和设置地图
//###################################################
void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." << std::endl;
  }

  grid = map;
  //update the configuration space with the current map
  configurationSpace.updateGrid(map);
  //create array for Voronoi diagram
//  ros::Time t0 = ros::Time::now();
  int height = map->info.height;
  int width = map->info.width;

  bool** binMap;
  binMap = new bool*[width];

  for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      binMap[x][y] = map->data[y * width + x] ? true : false;
    }
  }

  voronoiDiagram.initializeMap(width, height, binMap);
  voronoiDiagram.update();
  voronoiDiagram.visualize();
//  ros::Time t1 = ros::Time::now();
//  ros::Duration d(t1 - t0);
//  std::cout << "created Voronoi Diagram in ms: " << d * 1000 << std::endl;

  // plan if the switch is not set to manual and a transform is available
  if (!Constants::manual && listener.canTransform("/map", ros::Time(0), "/base_link", ros::Time(0), "/map", nullptr)) {

    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

    // assign the values to start from base_link
    start.pose.pose.position.x = transform.getOrigin().x();
    start.pose.pose.position.y = transform.getOrigin().y();
    tf::quaternionTFToMsg(transform.getRotation(), start.pose.pose.orientation);

    if (grid->info.height >= start.pose.pose.position.y && start.pose.pose.position.y >= 0 &&
        grid->info.width >= start.pose.pose.position.x && start.pose.pose.position.x >= 0) {
      // set the start as valid and plan
      validStart = true;
    } else  {
      validStart = false;
    }

    plan();
  }
}

//###################################################

//将当前姿态设置为起点
//                                   INITIALIZE START
//###################################################
// void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
//   float x = initial->pose.pose.position.x / Constants::cellSize;
//   float y = initial->pose.pose.position.y / Constants::cellSize;
//   float t = tf::getYaw(initial->pose.pose.orientation);
//   // publish the start without covariance for rviz
//   geometry_msgs::PoseStamped startN;
//   startN.pose.position = initial->pose.pose.position;
//   startN.pose.orientation = initial->pose.pose.orientation;
//   startN.header.frame_id = "map";
//   startN.header.stamp = ros::Time::now();
//   std::cout<<"start position"<<initial->pose.pose.position.x<<"  "<<initial->pose.pose.position.y<<" "<<initial->pose.pose.orientation;
//   std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

//   if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
//     validStart = true;
//     start = *initial;

//     //  start.pose.pose.position.x=54.6448;
//     //  start.pose.pose.position.y=28.8449;
//     //  start.pose.pose.orientation.x=0;
//     //  start.pose.pose.orientation.y=0;
//     //  start.pose.pose.orientation.z=0.138845;
//     //  start.pose.pose.orientation.w=0.990314;
//     //  std::cout<<"start position"<<start.pose.pose.position.x<<"  "<<start.pose.pose.position.y<<" "<<start.pose.pose.orientation;
//     if (Constants::manual) { plan();}

//     // publish start for RViz
//     pubStart.publish(startN);
//   } else {
//     std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
//   }
// }



//Traversability of start pose
//                                   INITIALIZE START
//###################################################
void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
  geometry_msgs::PoseWithCovarianceStamped initialCopy = *initial;
  // bool useFlag = Constants::use_flag;
  // if(useFlag){
  //    initialCopy.pose.pose.position.x = Constants::start_position_x;
  //    initialCopy.pose.pose.position.y =Constants::start_position_y;
  //    initialCopy.pose.pose.position.z = Constants::start_position_z;
  //    initialCopy.pose.pose.orientation.x = Constants::start_orientation_x;
  //    initialCopy.pose.pose.orientation.y = Constants::start_orientation_y;
  //    initialCopy.pose.pose.orientation.z =Constants::start_orientation_z;
  //    initialCopy.pose.pose.orientation.w = Constants::start_orientation_w;
  // }
  
  float x = initialCopy.pose.pose.position.x;
  float y = initialCopy.pose.pose.position.y;
  float t = tf::getYaw(initialCopy.pose.pose.orientation);
 

  std::ifstream fp;
  fp.open("/home/nubot/T-Hybrid-planner/src/data/terrainData.txt",std::ios::in);
  double voxel[5];
  std::vector<double> voldata;
  std::vector<double> child_data;
  Eigen::Vector3d Point_nomal;
  Eigen::Vector4f centroid;
  Eigen::Vector3f Point_position;
  Eigen::Vector3d car_angles;
  double pitch_angle,roll_angle;
  double traver_value;
  int ID;
//检测文件是否打开成功；
  if(!fp.is_open()){
        std::cout<<"Can not open the file!\n";
    }
    std::map<int,std::vector<double>> mymap;
    

  while(!fp.eof()) //feof（）检测一个文件是否结束，即到达文件尾，若结束，则返回非0值，否则返回0
  {
 
       fp>>ID>>voxel[0]>>voxel[1]>>voxel[2]>>voxel[3]>>voxel[4];
       for (int i = 0;i<5;i++)
       {
          voldata.push_back(voxel[i]);

       }

      mymap[ID]=voldata;
      voldata.clear();

  }
  fp.close();
  int car_X=(int)ceil(x/Constants::cellSize) - 1;   //ID均从0开始计算  round是四舍五入取法
  int car_Y=(int)ceil(y/Constants::cellSize) - 1;   
  int car_ID=(int)(car_X+car_Y*Constants::volnum);
  std::map<int,std::vector<double>>::iterator it;
  if(mymap.count(car_ID)){
    // std::cout<<"找到匹配的ID"<<car_X<<" "<<car_Y<<"  "<<car_ID<<std::endl;
        it=mymap.find(car_ID);
    //获取volxel内数据
          child_data=it->second;
          double dis_fac=child_data[1];
          Point_nomal(0)=child_data[2];
          Point_nomal(1)=child_data[3];
          Point_nomal(2)=child_data[4];
          car_angles=MyPcl::Car_Angle(Point_nomal,t);
          pitch_angle=car_angles(1);
          roll_angle=car_angles(2);
          double pitch_fac=std::max(pitch_angle/(-0.3),pitch_angle/0.25);   //上坡可以最大-0.3 下坡最大为0.25
          double roll_fac=std::abs(roll_angle)/0.18;     //滚转角最大0.18
          //At least one item exceeds its threshold
          if(pitch_fac>=1 || roll_fac>=1 || dis_fac>=1.3){  
                    std::cout<<"Exceed the threshold"<<std::endl;
                    std::cout<<pitch_fac<<"    "<<roll_fac<<"   "<<dis_fac;
            }
          traver_value=1-(0.05*pitch_fac+0.95* roll_fac+0.05*dis_fac);
          std::cout<<"static traversability is: "<<traver_value<<std::endl;
  }
  else{  //如果没有查询到的话，直接删除节点
    std::cout<<"There is no terrain data"<<car_X<<" "<<car_Y<<"  "<<car_ID<<std::endl;
  }

 // publish the start without covariance for rviz
  geometry_msgs::PoseStamped startN;
  startN.pose.position = initialCopy.pose.pose.position;
  startN.pose.orientation = initialCopy.pose.pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();
  std::cout<<"start position"<<initialCopy.pose.pose.position.x<<"  "<<initialCopy.pose.pose.position.y<<" "<<initialCopy.pose.pose.orientation;
  std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validStart = true;
    start = initialCopy;

    if (Constants::manual) { plan();}

    // publish start for RViz
    pubStart.publish(startN);
  } else {
    std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}









//###################################################
//                                    INITIALIZE GOAL
//###################################################
void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {


    geometry_msgs::PoseStamped endCopy = *end;
  //是否使用同一个终点 
  // bool useFlag = Constants::use_flag;
   
  // if(useFlag){
  //    endCopy.pose.position.x = Constants::end_position_x;
  //    endCopy.pose.position.y = Constants::end_position_y;
  //    endCopy.pose.position.z = Constants::end_position_z;
  //    endCopy.pose.orientation.x = Constants::end_orientation_x;
  //    endCopy.pose.orientation.y = Constants::end_orientation_y;
  //    endCopy.pose.orientation.z = Constants::end_orientation_z;
  //    endCopy.pose.orientation.w = Constants::end_orientation_w;
  // }


 // retrieving goal position
  float x = endCopy.pose.position.x / Constants::cellSize;
  float y = endCopy.pose.position.y / Constants::cellSize;
  float t = tf::getYaw(endCopy.pose.orientation);

  std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validGoal = true;
    goal = endCopy;

      std::cout<<"end position"<<goal.pose.position.x<<"  "<<goal.pose.position.y<<" "<<goal.pose.orientation;

    if (Constants::manual) { plan();}
    pubGoalpose.publish(goal);

  } else {
    std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}





//###################################################
//                                      PLAN THE PATH
//###################################################
void Planner::plan() {
  // if a start as well as goal are defined go ahead and plan
  if (validStart && validGoal) {

    // ___________________________
    // LISTS ALLOWCATED ROW MAJOR ORDER
    int width = grid->info.width;
    int height = grid->info.height;


    int depth = Constants::headings;
    int length = width * height * depth;
    // define list pointers and initialize lists
    Node3D* nodes3D = new Node3D[length]();
    Node2D* nodes2D = new Node2D[width * height]();

    // ________________________
    // retrieving goal position
    float x = goal.pose.position.x / Constants::cellSize;
    float y = goal.pose.position.y / Constants::cellSize;
    float t = tf::getYaw(goal.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    const Node3D nGoal(x, y, t, 0, 0, nullptr);
    // __________
    // DEBUG GOAL
    //    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);


    // _________________________
    // retrieving start position
    x = start.pose.pose.position.x / Constants::cellSize;
    y = start.pose.pose.position.y / Constants::cellSize;
    t = tf::getYaw(start.pose.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    Node3D nStart(x, y, t, 0, 0, nullptr);
    // ___________
    // DEBUG START
    //    Node3D nStart(108.291, 30.1081, 0, 0, 0, nullptr);


    // ___________________________
    // START AND TIME THE PLANNING
    ros::Time t0 = ros::Time::now();

    // CLEAR THE VISUALIZATION
    visualization.clear();
    // CLEAR THE PATH
    path.clear();
    smoothedPath.clear();
    // FIND THE PATH  
    Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace, dubinsLookup, visualization);
    // TRACE THE PATH
    smoother.tracePath(nSolution);
    // CREATE THE UPDATED PATH
    path.updatePath(smoother.getPath());
    // SMOOTH THE PATH
    smoother.smoothPath(voronoiDiagram);
    // CREATE THE UPDATED PATH
    smoothedPath.updatePath(smoother.getPath());
    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    std::cout << "TIME in ms: " << d * 1000 << std::endl;
    

    // _________________________________
    // PUBLISH THE RESULTS OF THE SEARCH
    path.publishPath();
    path.publishPathNodes();
    path.publishPathVehicles();
    smoothedPath.publishPath();
    smoothedPath.publishPathNodes();
    smoothedPath.publishPathVehicles();
    visualization.publishNode3DCosts(nodes3D, width, height, depth);
    visualization.publishNode2DCosts(nodes2D, width, height);



    delete [] nodes3D;
    delete [] nodes2D;
    this->replan_=false;

  } else {
    std::cout << "missing goal or start" << std::endl;
  }
}
