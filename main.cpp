#include <ros/ros.h>
#include <rosThread.h>


int main(int argc,char** argv){

    ros::init(argc,argv,"pos_calculator");
    RosThread* rosthread  = new RosThread();
    rosthread->work();




}
