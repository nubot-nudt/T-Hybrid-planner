 Our path planner is inspired by [Kurzer' Hybrid A* algorithm](https://github.com/karlkurzer/path_planner.git), and some improvements are made based on this algorithm. 

 Please modify the corresponding parameters in `constants.h`
```c++
///T-hybrid A* paramaters
static const float gridSize = 0.3; //The voxel grid size
static const float mapLength = 27; //Length of the map
static const float traverPenality = 4.0; //Proportions parameter of terrain passable constraint
static const float TurnPenality = 1.05; //Proportions parameter of turning constraint
static const int  volnum= (int)round(mapLength/gridSize); //The number of voxel grids per row in the map
static const float thetaYmin = -0.3;   // Min pitch angle 
static const float thetaYmax = 0.25;   // Max pitch angle
static const float thetaXmax = 0.18;   // Min pitch angle
static const float pitchWeight = 0.4;  //Pitch angle weight
static const float rollWeight  = 0.4; //Roll angle weight
static const float roughWeight = 0.3; //roughness weight
```

