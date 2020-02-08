/*
 File name: rosThread.h
 Author: Berkay Gümüş
 E-mail: berkay.gumus@boun.edu.tr
 Date created: 07.01.2020
 Date last modified: 07.01.2020
 */

#include "ros/ros.h"
#include <geometry_msgs/PoseArray.h>
#include <turtlesim/Pose.h>
#include <math.h>
#include <cfloat>
#define PI 3.14159265359
#define LOOP_RATE 10

#define N 3
#define SIZE_P 100 //length of pos data, 100 data for 10 second, f=10 hz
#define SIZE_T 3 //length of time data, 3 last seen time data

using namespace std;

class RosThread{
  public:

    RosThread();
    ~RosThread();
    void work();

  private:


    ros::NodeHandle sensorSub;
    ros::NodeHandle posPub;


    void sensorCallback0(const geometry_msgs::PoseArray::ConstPtr&);
    void sensorCallback1(const geometry_msgs::PoseArray::ConstPtr&);
    void sensorCallback2(const geometry_msgs::PoseArray::ConstPtr&);
    void sensorCallback3(const geometry_msgs::PoseArray::ConstPtr&);
    void sensorCallback4(const geometry_msgs::PoseArray::ConstPtr&);
    void sensorCallback5(const geometry_msgs::PoseArray::ConstPtr&);
    void sensorCallback6(const geometry_msgs::PoseArray::ConstPtr&);

    int n = N;
    int sizeP = SIZE_P;
    int sizeT = SIZE_T;
    int t=0;
    int index_t = 0;
    double robotsSeen[SIZE_P][N][N][2];
    int lastSeen[N][N][SIZE_T];
    double angles[N][N];

    void addSeenTime(int i, int j);
    bool checkSeenTime(int i, int j);
    void checkLastSeen();
    void calculateTransform();




};
