#ifndef ROS_RLS_HPP_
#define ROS_RLS_HPP_


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "cisst_msgs/rlsEstData.h"
#include "cisst_msgs/vctDoubleVec.h"
#include "sensor_msgs/JointState.h"
#include "RLSestimator.h"
#include <sstream>
#include <vector>
#include <iostream>
#include <fstream>
class rosRLS{ 
	
    public:

       
        ros::NodeHandle nh_;
        ros::Subscriber jointAnglesub_;
        ros::Subscriber wrenchsub_;
        ros::Publisher rosRLSpub_;
	    ros::Publisher Fnpub_;
	    ros::Publisher Ftpub_;
	  
        vctDynamicVector<double> currJointPos;
        vctDoubleVec currWrench;

        vctDoubleVec rlsEstData;
        cisst_msgs::rlsEstData rlsEstData_msg;
        RLSestimator* rls;
        
        std::ofstream ofsForceData;
        double startTime;
	
	std_msgs::Float64 Fn_;
	std_msgs::Float64 Ft_;
	
        rosRLS(ros::NodeHandle& nh, std::string jointAngleTopic, std::string wrenchTopic)
        {
            nh_ = nh;
            
            // initialize subscriber and publisher
            jointAnglesub_ = nh_.subscribe<sensor_msgs::JointState>(jointAngleTopic, 1000, &rosRLS::JointStateCallback,this); 
            wrenchsub_ = nh_.subscribe<cisst_msgs::vctDoubleVec>(wrenchTopic, 1000, &rosRLS::WrenchCallback,this); 

            rosRLSpub_ = nh_.advertise<cisst_msgs::rlsEstData>("/rlsEstData", 1000);  
	        Fnpub_ = nh_.advertise<std_msgs::Float64>("/Fn", 1000);
	        Ftpub_ = nh_.advertise<std_msgs::Float64>("/Ft", 1000);
	      
            startTime = ros::Time::now().toSec();
            Eigen::Vector2d xinit(0.58, 4);
            rls = new RLSestimator(nh_, xinit, startTime);

            rlsEstData.SetSize(9);
            rlsEstData.Assign((double)0,0,0,0,0,0,0,0,0);
	    
	    currWrench.SetSize(6);
	    currWrench.Assign((double)0,0,0,0,0,0);
	    currJointPos.SetSize(7);
	    currJointPos.Assign((double)0,0,0,0,0,0,0);

        ofsForceData.open("RLS.txt");
        }

        ~rosRLS(void){}

	void JointStateCallback(const sensor_msgs::JointState::ConstPtr& jointAngle_msg)
    {
        
        currJointPos[0] = jointAngle_msg->position[0];
        currJointPos[1] = jointAngle_msg->position[1];
        currJointPos[2] = jointAngle_msg->position[2];
        currJointPos[3] = jointAngle_msg->position[3];
        currJointPos[4] = jointAngle_msg->position[4];
        currJointPos[5] = jointAngle_msg->position[5];
        currJointPos[6] = jointAngle_msg->position[6];
    }

    void WrenchCallback(const cisst_msgs::vctDoubleVec::ConstPtr& wrench_msg)
    {
        
        currWrench[0] = wrench_msg->data[0];
        currWrench[1] = wrench_msg->data[1];
        currWrench[2] = wrench_msg->data[2];
        currWrench[3] = wrench_msg->data[3];
        currWrench[4] = wrench_msg->data[4];
        currWrench[5] = wrench_msg->data[5];

    }

    void rlsPublish()
    {
        double currtime = ros::Time::now().toSec() - startTime;
        
        double Fn = currWrench[2];
        double Fe = currWrench[0];

	Fn_.data = Fn;
	Ft_.data = Fe;
             /** parameters: 
                *
                * Fn: normal force as measured by the force sensor (currently force in the z direction)
                * Fe: tangential force as measured by the force sensor (currently force in the x direction)
                * currtime: current time = osaGetTime() - startTime
                * currJointPos: current joint positions for all 7 joints
                */

           rls->RLSestimate(Fn, 
                            Fe,
                            currtime,
	            		    currJointPos);
	   
	   rls->GetEstimates(rlsEstData);


        rlsEstData_msg.F0 = rlsEstData[0];
        rlsEstData_msg.Fn = rlsEstData[1];
        rlsEstData_msg.mu = rlsEstData[2];
        rlsEstData_msg.Fc = rlsEstData[3];
        rlsEstData_msg.Fe = rlsEstData[4];
        rlsEstData_msg.Fnobs = rlsEstData[5];
        rlsEstData_msg.haveFailed = rlsEstData[6];
        rlsEstData_msg.haveTeared = rlsEstData[7];
        rlsEstData_msg.haveSlided = rlsEstData[8];
        
	rosRLSpub_.publish(rlsEstData_msg);
	Fnpub_.publish(Fn_);
	Ftpub_.publish(Ft_);


        ofsForceData << currtime << "," 
                     << rlsEstData_msg.F0 << ","
                     << rlsEstData_msg.Fn << ","
                     << rlsEstData_msg.mu << ","
                     << rlsEstData_msg.Fc << ","
                     << rlsEstData_msg.Fe << ","
		             << rlsEstData_msg.haveFailed << ","
                     << rlsEstData_msg.haveTeared << ","
                     << rlsEstData_msg.haveSlided << ","
                     << rlsEstData[8] << ","<<std::endl;

    }    
};      

#endif



