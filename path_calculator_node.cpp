/*===============================================================================
|*Author: Jose Benitez
|*Date: October 2016
|*Description: Calculate the shortest path between two points on a map using the 
|*             A* algorithm.
\*===============================================================================*/

#include <iostream>
#include "ros/ros.h"
#include "navig_msgs/CalculatePath.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <climits>
#include <ctime>
#include <vector>

#define abs(x) (x<0?-x:x)
#define max(a,b)  (a>b?a:b)
#define CONECTIVIDAD  8

#define MAX_DILATE    6
#define ALPHA         0.1        //Difference weight (Original path)
#define BETA          0.9        //Distance weight (straight line)
#define DELTA         0.05
#define TOL           0.001

//Main functions declaration
bool dilatar(nav_msgs::OccupancyGrid mapa, nav_msgs::OccupancyGrid * mapa_dilatado);
bool calculatePath(geometry_msgs::Pose start, geometry_msgs::Pose goal,
                   nav_msgs::OccupancyGrid& mapa, nav_msgs::Path * ruta);
bool suavizar(nav_msgs::Path ruta, nav_msgs::Path * ruta_suavizada);

//A* functions
unsigned int xy2idx(unsigned int x, unsigned int y, unsigned int w);
unsigned int heuristics(unsigned int current, unsigned int goal, unsigned int W);
void calculateVecinos(int* vecinos, unsigned int idx, unsigned int W, unsigned int H);
int idxOfmin(int* f_vals);

//Smoothing functions
bool deltaV(nav_msgs::Path pathP, nav_msgs::Path pathQ, float dV[][2]);
float norm2(float m[][2], int n);

//This is what we want :D
nav_msgs::Path solved_path;
std::vector<int> open_list;

//Callback from the service
bool calculate_path(navig_msgs::CalculatePath::Request &req,
                    navig_msgs::CalculatePath::Response &res){
    std::cout << "Executing service..." << std::endl;

//Useful info
    float x_i, y_i, x_f, y_f;    
    x_i = req.start_pose.position.x; y_i = req.start_pose.position.y;
    x_f = req.goal_pose.position.x; y_f = req.goal_pose.position.y;
    std::cout << "Start pose: " << x_i << " " << y_i << std::endl;
    std::cout << "Goal pose:  " << x_f << " " << y_f << std::endl;

//Main functions
    nav_msgs::OccupancyGrid mapa_dilatado;
    if(!dilatar(req.map, &mapa_dilatado)){
      std::cout << "ERROR: Could not dilate the map" << std::endl;
      return false;
    }

    nav_msgs::Path ruta_cruda;
    if(!calculatePath(req.start_pose,req.goal_pose,mapa_dilatado,&ruta_cruda)){
      std::cout << "ERROR: Could not calculate path" << std::endl;
      return false;
    }

    if(!suavizar(ruta_cruda, &solved_path)){
      std::cout << "ERROR: Could not smooth path" << std::endl;
      return false;
    }

    res.path = solved_path;
    std::cout << "Ready" << std::endl;
    return true;
}



//Funcion para dilatar
bool dilatar(nav_msgs::OccupancyGrid mapa, nav_msgs::OccupancyGrid * mapa_dilatado){
  std::cout << "Dilating... ";
  *mapa_dilatado = mapa;
  unsigned int W = mapa.info.width;
  for(int j=1; j<= MAX_DILATE; j++){
    for(int i=0;i<W*mapa.info.height; i++){
      if(int(mapa.data[i]) == 100){
        (*mapa_dilatado).data[i+1] = int(100);
        (*mapa_dilatado).data[i+W+1] = int(100);
        (*mapa_dilatado).data[i+W] = int(100);
        (*mapa_dilatado).data[i+W-1] = int(100);
        (*mapa_dilatado).data[i-1] = int(100);
        (*mapa_dilatado).data[i-W-1] = int(100);
        (*mapa_dilatado).data[i-W] = int(100);
        (*mapa_dilatado).data[i-W+1] = int(100);
      }
    }
    mapa = *mapa_dilatado;
  }
  std::cout << "Map dilated!" << std::endl;
  return true;
}



//Funcion para calcular la ruta mas corta usando A*
bool calculatePath(geometry_msgs::Pose start, geometry_msgs::Pose goal,
                   nav_msgs::OccupancyGrid& mapa, nav_msgs::Path * ruta){

  std::cout << "Calculating path... " << std::endl;

  unsigned int W = mapa.info.width;
  unsigned int H = mapa.info.height;

  const float res = mapa.info.resolution;
  const int oriX = mapa.info.origin.position.x/res; //-999
  const int oriY = mapa.info.origin.position.y/res; //-399

  const unsigned int max_idx = W*H;
  unsigned int attempts = 0;
  const unsigned int max_attempts = max_idx;

  int* g_values = new int[max_idx];
  bool* isknown = new bool[max_idx];
  int* previous = new int[max_idx];
  int* f_values = new int[max_idx];

  unsigned int g_temp;

  for(int i=0; i<max_idx; i++){
    g_values[i] = UINT_MAX;
    isknown[i] = false;
    previous[i] = -1;
    f_values[i] = UINT_MAX;
  }

  unsigned int start_cell = xy2idx(start.position.x/res-oriX, start.position.y/res-oriY, W);
  unsigned int goal_cell = xy2idx(goal.position.x/res-oriX, goal.position.y/res-oriY, W);
  
  if(int(mapa.data[start_cell])!=0){
    std::cout << "ERROR: Robot cant start from that point on the dilated map." << std::endl;
    return false;
  }
  if(int(mapa.data[goal_cell])!=0){
    std::cout << "ERROR: Robot cant reach that point on the dilated map." << std::endl;
    return false;
  }

  int current_cell = start_cell;

  g_values[current_cell] = 0;
  f_values[current_cell] = 0 + heuristics(current_cell, goal_cell, W);
  isknown[current_cell] = true;

  clock_t begin = clock();
//AQUI VIENE EL PEZ GORDO
  int vecinos[CONECTIVIDAD];
  while(current_cell != goal_cell && attempts < max_attempts){
    attempts++;
    
    calculateVecinos(vecinos, current_cell, W, H);

    for(int n = 0; n < CONECTIVIDAD; n++){
      if(!isknown[vecinos[n]] && int(mapa.data[vecinos[n]])==0){
        if(CONECTIVIDAD == 8 && n >= 4)
          g_temp = g_values[current_cell] + 14;        
        else
          g_temp = g_values[current_cell] + 10;

        if(g_temp < g_values[vecinos[n]]){
          g_values[vecinos[n]] = g_temp;
          f_values[vecinos[n]] = g_temp + heuristics(vecinos[n], goal_cell, W);
          previous[vecinos[n]] = current_cell;
          open_list.push_back(vecinos[n]);
        }
      }//if a valid neighbor
    }//for all neighbors

    current_cell = idxOfmin(f_values);
    if(current_cell == -2){
      std::cout << "ERROR: Could not find min value on f_values!" << std::endl;
      std::cout << "(Maybe there is no way, or robot too fat)" << std::endl;
      delete[] g_values;  delete[] isknown;
      delete[] previous;  delete[] f_values;
      return false;
    }
    isknown[current_cell] = true;
  }

  if(attempts >= max_attempts){
    std::cout << "ERROR: Max attempts reached (Path not found)" << std::endl;
    delete[] g_values;  delete[] isknown;
    delete[] previous;  delete[] f_values;
    return false;
  }

  std::cout << "Elapsed time: " << float(clock() - begin) / CLOCKS_PER_SEC << std::endl;
  std::cout << "Attempts: " << attempts << std::endl;
  geometry_msgs::PoseStamped myNewPose;
  while(current_cell != -1){
    myNewPose.pose.position.x = int((current_cell % W)+oriX)*res;
    myNewPose.pose.position.y = int((current_cell / W)+oriY)*res;
    myNewPose.pose.orientation.w = 1;//Just in case
    myNewPose.header.frame_id = "map";
    (*ruta).poses.push_back(myNewPose);
    current_cell = previous[current_cell];
  }

  (*ruta).header.frame_id = "map";

  delete[] g_values;  delete[] isknown;
  delete[] previous;  delete[] f_values;

  std::cout << "Path calculated successfully" << std::endl;
  return true;
}

unsigned int xy2idx(unsigned int x, unsigned int y, unsigned int w){
  return (w*(y+1)) - (w-x);
}

unsigned int heuristics(unsigned int current, unsigned int goal, unsigned int W){
  //return abs(goal % W - current % W)+abs(goal / W - current / W);
  return max(abs(goal % W - current % W), abs(goal / W - current / W));
}

void calculateVecinos(int* vecinos, unsigned int idx, unsigned int W, unsigned int H){
  if(idx >= W*H) return;
  vecinos[0] = idx - W;  //Upper neighbor
  vecinos[1] = idx + 1;  //Rigth neighbor
  vecinos[2] = idx + W;  //Lower neighbor
  vecinos[3] = idx - 1;  //Left neighbor
#if CONECTIVIDAD == 8
  vecinos[4] = idx - W + 1;  //
  vecinos[5] = idx - W - 1;  //
  vecinos[6] = idx + W + 1;  //
  vecinos[7] = idx + W - 1;  //
#endif
}

int idxOfmin(int* f_vals){
  int pos=-2, lastj = -1;
  unsigned int min=UINT_MAX;
  for(int j=0;j<int(open_list.size());j++){
    int i = open_list.at(j);
    if(f_vals[i]<min){
      pos=i;
      lastj=j;
      min=f_vals[i];
    }
  }
  open_list.erase(open_list.begin()+lastj);
  return pos;
}

//Funcion para suavizar la ruta
bool suavizar(nav_msgs::Path ruta, nav_msgs::Path * ruta_suavizada){
  std::cout << "Smoothing...";
  *ruta_suavizada = ruta;

///////////////////////[UNDER CONSTRUCTION]///////////////////////
  int n = int(ruta.poses.size());
  float dV[n][2];
  deltaV(*ruta_suavizada, ruta, dV);  //Aqui ambas rutas son iguales
  clock_t begin = clock();

  while(norm2(dV, n) > TOL){
    for(int i=0; i<n; i++){
      (*ruta_suavizada).poses[i].pose.position.x += (-DELTA*dV[i][0]);
      (*ruta_suavizada).poses[i].pose.position.y += (-DELTA*dV[i][1]);
    }
    deltaV(*ruta_suavizada, ruta, dV);
  }
///////////////////////[END OF CONSTRUCTION]///////////////////////
  std::cout << "Elapsed time: " << float(clock() - begin) / CLOCKS_PER_SEC << std::endl;
  std::cout << "Path smoothed" << std::endl;
  return true;
}

bool deltaV(nav_msgs::Path pathP, nav_msgs::Path pathQ, float dV[][2]){
///////////////////////[UNDER CONSTRUCTION]///////////////////////
  float px, py, qx, qy, pxl, pxn, pyl, pyn;
  int n = int(pathQ.poses.size());

  for(int i=1; i<n-1; i++){
    px = pathP.poses[i].pose.position.x;
    py = pathP.poses[i].pose.position.y;
    qx = pathQ.poses[i].pose.position.x;
    qy = pathQ.poses[i].pose.position.y;
    pxl = pathP.poses[i-1].pose.position.x;
    pxn = pathP.poses[i+1].pose.position.x;
    pyl = pathP.poses[i-1].pose.position.y;
    pyn = pathP.poses[i+1].pose.position.y;
    
    dV[i][0] = ALPHA*(px-qx)+BETA*(2*px-pxl-pxn);
    dV[i][1] = ALPHA*(py-qy)+BETA*(2*py-pyl-pyn);
  }

//First term
  px = pathP.poses[0].pose.position.x;
  py = pathP.poses[0].pose.position.y;
  qx = pathQ.poses[0].pose.position.x;
  qy = pathQ.poses[0].pose.position.y;
  pxn = pathP.poses[1].pose.position.x;
  pyn = pathP.poses[1].pose.position.y;
  
  dV[0][0] = ALPHA*(px-qx)+BETA*(px-pxn);
  dV[0][1] = ALPHA*(py-qy)+BETA*(py-pyn);

//Last term
  px = pathP.poses[n-1].pose.position.x;
  py = pathP.poses[n-1].pose.position.y;
  qx = pathQ.poses[n-1].pose.position.x;
  qy = pathQ.poses[n-1].pose.position.y;
  pxl = pathP.poses[n-2].pose.position.x;
  pyl = pathP.poses[n-2].pose.position.y;
  
  dV[n-1][0] = ALPHA*(px-qx)+BETA*(px-pxl);
  dV[n-1][1] = ALPHA*(py-qy)+BETA*(py-pyl);
///////////////////////[END OF CONSTRUCTION]///////////////////////
  return true;
}

float norm2(float m[][2], int n){
  float sum = 0;
  for(int i=0; i<n; i++){
    sum += (m[i][0]*m[i][0] + m[i][1]*m[i][1]);
  }
  return sqrt(sum);
}

int main (int argc, char ** argv){

  std::cout<<">Initializing path calculator node..." <<std::endl;
  ros::init(argc,argv,"path_calculator");
  ros::NodeHandle n;
  ros::Publisher pubPath;

  ros::Rate loop(10);

  ros::ServiceServer srvAstar = n.advertiseService("navigation/a_star", calculate_path);
  pubPath = n.advertise<nav_msgs::Path>("/navigation/a_star_path",1);


  while(ros::ok()){
    pubPath.publish(solved_path);
    ros::spinOnce();
    loop.sleep();
  }
  return 0;
}

/*
geometry_msgs/Pose start_pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
geometry_msgs/Pose goal_pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
nav_msgs/OccupancyGrid map
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  nav_msgs/MapMetaData info
    time map_load_time
    float32 resolution
    uint32 width
    uint32 height
    geometry_msgs/Pose origin
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
  int8[] data
---
nav_msgs/Path path
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  geometry_msgs/PoseStamped[] poses
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Pose pose
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
//==========================
*/
