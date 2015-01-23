#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rosRLS.h"
#include <sstream>

int main(int argc, char **argv)
{


  ros::init(argc, argv, "RLSestimator");


  ros::NodeHandle n;
  

  rosRLS rlsPub( n, "/pid/joint_current", "/logger/MsrFT" );

  ros::Rate loop_rate(100);


  int count = 0;
    while (n.ok())
     {
        
       rlsPub.rlsPublish();     
        ros::spinOnce();
        loop_rate.sleep();
        ++count;

     }
 
  return 0;

}



 


