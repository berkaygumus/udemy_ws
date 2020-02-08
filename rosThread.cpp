#include "rosThread.h"
#include <iostream>
#include <math.h>
#include <string>
#include <sstream>
#include <geometry_msgs/PoseArray.h>

using namespace std;



RosThread::RosThread(){
  for(int tt=0;tt<sizeP;tt++){
    for(int i=0;i<n;i++){
      for(int j=0;j<n;j++){
        for(int k=0;k<2;k++){
          robotsSeen[tt][i][j][k] = -1000;
        }
      }
    }
  }

  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      for(int k=0;k<sizeT;k++){
        lastSeen[i][j][k] = -1;
      }
    }
  }

  cout << "pos_calculator is created!" << endl;
}

RosThread::~RosThread(){

}

void RosThread::sensorCallback0(const geometry_msgs::PoseArray::ConstPtr& msg){
  //cout << "callback0: " << t << endl;
  for(int j=0;j<n;j++){
    robotsSeen[index_t][0][j][0] = msg->poses[j].position.x;
    robotsSeen[index_t][0][j][1] = msg->poses[j].position.y;
    //cout << "call 0-->" << j << " " << msg->poses[j].position.x << " " << msg->poses[j].position.y << endl;

    if(checkSeenTime(0,j)){
        addSeenTime(0,j);
    }
  }


}

void RosThread::sensorCallback1(const geometry_msgs::PoseArray::ConstPtr& msg){
  //cout << "callback1: " << t << endl;
  for(int j=0;j<n;j++){
    robotsSeen[index_t][1][j][0] = msg->poses[j].position.x;
    robotsSeen[index_t][1][j][1] = msg->poses[j].position.y;
    //cout << "1-->" << j << " " << msg->poses[j].position.x << " " << msg->poses[j].position.y << endl;
    if(checkSeenTime(1,j)){
        addSeenTime(1,j);
    }
  }

}

void RosThread::sensorCallback2(const geometry_msgs::PoseArray::ConstPtr& msg){
  for(int j=0;j<n;j++){
    robotsSeen[index_t][2][j][0] = msg->poses[j].position.x;
    robotsSeen[index_t][2][j][1] = msg->poses[j].position.y;
    //cout << "2-->" << j << " " << msg->poses[j].position.x << " " << msg->poses[j].position.y << endl;
    if(checkSeenTime(2,j)){
        addSeenTime(2,j);
    }
  }
}

void RosThread::sensorCallback3(const geometry_msgs::PoseArray::ConstPtr& msg){

}

void RosThread::sensorCallback4(const geometry_msgs::PoseArray::ConstPtr& msg){

}

void RosThread::sensorCallback5(const geometry_msgs::PoseArray::ConstPtr& msg){

}

void RosThread::sensorCallback6(const geometry_msgs::PoseArray::ConstPtr& msg){

}

void RosThread::addSeenTime(int i, int j){//it's valid only for SIZE_T 3
  if(lastSeen[i][j][0] == -1){
    lastSeen[i][j][0] = t;
  }

  else if(lastSeen[i][j][1] == -1){
    lastSeen[i][j][1] = lastSeen[i][j][0];
    lastSeen[i][j][0] = t;
  }

  else{
    lastSeen[i][j][2] = lastSeen[i][j][1];
    lastSeen[i][j][1] = lastSeen[i][j][0];
    lastSeen[i][j][0] = t;
  }

}

bool RosThread::checkSeenTime(int i, int j){
  //cout << "index t: " << index_t << endl;
  if(robotsSeen[index_t][i][j][0]==-1000){
    return false;
  }

  int prev_t = lastSeen[i][j][0];

  if(prev_t == -1){
    return true;
  }

  else{
    int index_prev_t = prev_t%sizeP;
    double prev_x = robotsSeen[index_prev_t][i][j][0];
    double prev_y = robotsSeen[index_prev_t][i][j][1];
    double x = robotsSeen[index_t][i][j][0];
    double y = robotsSeen[index_t][i][j][1];
    double dist = pow(x-prev_x,2) + pow(y-prev_y,2);
    if(dist>1){
      return true;
    }
    else{
      return false;
    }
  }

}

void RosThread::checkLastSeen(){
  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      if(lastSeen[i][j][0] !=-1){
        for(int k=0;k<sizeT;k++){
          if(t-lastSeen[i][j][k] > sizeP-1){
            lastSeen[i][j][k] = -1;
          }
        }
      }
    }
  }
}

void RosThread::calculateTransform(){
  double dxi,dyi,dxj,dyj,thetai,thetaj,theta,dx,dy;
  int t2,t1;
  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      if(lastSeen[i][j][1] !=-1){
        t2 = lastSeen[i][j][0];
        t1 = lastSeen[i][j][1];
        int index_t2 = t2%sizeP;
        int index_t1 = t1%sizeP;

        dxi = robotsSeen[index_t2][i][j][0]-robotsSeen[index_t1][i][j][0];
        dyi = robotsSeen[index_t2][i][j][1]-robotsSeen[index_t1][i][j][1];
        dxj = robotsSeen[index_t2][j][j][0]-robotsSeen[index_t1][j][j][0];
        dyj = robotsSeen[index_t2][j][j][1]-robotsSeen[index_t1][j][j][1];
        thetai = atan2(dyi,dxi);
        thetaj = atan2(dyj,dxj);
        theta = thetai-thetaj;
        angles[i][j] = theta;
        angles[j][i] = -theta;

        dx = robotsSeen[index_t2][i][j][0] - robotsSeen[index_t2][j][j][0]*cos(theta) + robotsSeen[index_t2][j][j][1]*sin(theta);
        dy = robotsSeen[index_t2][i][j][1] - robotsSeen[index_t2][j][j][0]*sin(theta) - robotsSeen[index_t2][j][j][1]*cos(theta);


        cout << i << "-->" << j << " theta " << theta << " d: " << dx<< " "  << dy << endl;

      }
    }
  }
}


void RosThread::work(){


  /*ros::Publisher pos_pub0 = posPub.advertise<geometry_msgs::PoseArray>("/pos0", 100);
  ros::Publisher pos_pub1 = posPub.advertise<geometry_msgs::PoseArray>("/pos1", 100);
  ros::Publisher pos_pub2 = posPub.advertise<geometry_msgs::PoseArray>("/pos2", 100);*/

  ros::Subscriber sensor_sub1 = sensorSub.subscribe("/pos0",1000,&RosThread::sensorCallback0,this);
  ros::Subscriber sensor_sub2 = sensorSub.subscribe("/pos1",1000,&RosThread::sensorCallback1,this);
  ros::Subscriber sensor_sub3 = sensorSub.subscribe("/pos2",1000,&RosThread::sensorCallback2,this);
  ros::Subscriber sensor_sub4 = sensorSub.subscribe("/pos3",1000,&RosThread::sensorCallback3,this);
  ros::Subscriber sensor_sub5 = sensorSub.subscribe("/pos4",1000,&RosThread::sensorCallback4,this);
  ros::Subscriber sensor_sub6 = sensorSub.subscribe("/pos5",1000,&RosThread::sensorCallback5,this);
  ros::Subscriber sensor_sub7 = sensorSub.subscribe("/pos6",1000,&RosThread::sensorCallback6,this);



  ros::Rate loop_rate(10);


  while (ros::ok()){
    checkLastSeen();
    index_t = t%sizeP;
    cout << "time: " << t << endl;
    ros::spinOnce();

    loop_rate.sleep();

    for(int tt=0;tt<3;tt++){
      cout << "last seen: " << tt << endl;
      for(int i=0;i<n;i++){
        cout << "i: ";
        for(int j=0;j<n;j++){
          cout << lastSeen[i][j][tt] << " ";
        }
        cout << endl;
      }

    }
    calculateTransform();
    t++;
    cout << endl;
  }



}
