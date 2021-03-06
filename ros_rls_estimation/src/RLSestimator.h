#ifndef _RLS_ESTIMATOR_H
#define _RLS_ESTIMATOR_H

#include<iostream>
#include<cmath>
#include<vector>
#include<cisstCommon/cmnPath.h>
#include<cisstVector.h>
#include<algorithm>
#include<eigen3/Eigen/Dense>
#include<eigen3/Eigen/LU>
#include<dynamic_reconfigure/server.h>
#include<ros_rls_estimation/rlsConfig.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

class RLSestimator{

    public:
 
        // x[0] = mu is the cofficient of kinetic friction, x[1] = Fc is the estimated cutting force
        Eigen::Vector2d x;
	Eigen::Vector2d xlast; // used for motionDetection feature
	Eigen::Vector2d xLastSuccessful;

        // P is the covariance of x        
	Eigen::Matrix2d P;
	Eigen::Matrix2d Plast;
	Eigen::Matrix2d PlastSuccessful;

	ros::Subscriber paramTuneSub_;

	double lamda_Fc;
	double lamda_mu;

	// used to measure observability of input signal (Fn)
	std::vector<double> FnVec;
	std::vector<double> timeVec;

	double currTime;
        // Fest is the estimated total tangential force 
        double Fest;
        double FestLast; // used for motionDetection feature
	
        std::vector< std::vector<double> > jointPoses;
        std::vector<double> timeStamps;
        int avgNum;// number of joint position measurement to take in calculation of velocity

        //failstate: used to indicate cutting failure (true if failed)
        bool failstate;
        // indicate contate state
        bool contactState;
        //wamNotMoving: is true when wam is not moving
        bool wamNotMoving;
        // true if wam is not moving;
        bool wamMotionState;
        
        // tunable threholds
        double cuttingFailureThreshold; // defines when cutting failure happens
	double slidingThreshold; // defines when sliding without cutting happens
        double contactThreshold;//
	double tearingThreshold;
	double FnobservatbilityThreshold;
	double motionThreshold;

	int windowSize;
        //rlsEstData: is a vector containing [Fe, Fn, mu, Fc, Fest, haveFailed, haveTeared, haveSlipped] that gets pushed to ROS (gets changed by the method)
        vctDoubleVec rlsEstData; // for ROS
        //haveFailed: a double value used to draw a spike on rqt if failure happens (set to a large number)
        double haveFailed; // for cisst_msgs::rlsEstData
	double haveTeared;
	double haveSlided;

	double avgJointVel;
        // constructor
 RLSestimator(ros::NodeHandle nh, Eigen::Vector2d& xinit, double& startTime): 
        
	x(xinit),
	  xlast(xinit),
	  xLastSuccessful(xinit),
	  P(Eigen::Matrix2d::Identity()),
	  currTime(startTime),
	  Plast(Eigen::Matrix2d::Identity()),   
	  PlastSuccessful(Eigen::Matrix2d::Identity()),
	  failstate(false),
	  avgNum(100),
	  contactState(true),
	  wamMotionState(true),
	  wamNotMoving(true),  
	  cuttingFailureThreshold(1.8), // tune this
	  slidingThreshold(1.8),        // tune this
	  contactThreshold(0.05),       // tune this
	  tearingThreshold(1.8),        // tune this
	  FnobservatbilityThreshold(0.3),//tune this
	  lamda_Fc(0.985),              // tune this 
	  lamda_mu(0.99),               // tune this
	  motionThreshold(0.001),       // tune this
	  windowSize(80),               // tune this
	  Fest(0),
	  FestLast(0),
	  haveFailed(0),
	  haveTeared(0),
	  haveSlided(0),
	  avgJointVel(0){
	  
	  timeStamps.push_back(startTime);

	  // initialize jointPoses
	  std::vector<double> temp(7, 0);
	  for(int i = 0; i < avgNum; i++){jointPoses.push_back(temp);} 

	  FnVec.push_back(0);
	  timeVec.push_back(startTime);
	  
	  rlsEstData.SetSize(9);
	  rlsEstData.Assign((double)0,0,0,0,0,0,0,0,0);

	  dynamic_reconfigure::Server<ros_rls_estimation::rlsConfig> server;
	  dynamic_reconfigure::Server<ros_rls_estimation::rlsConfig>::CallbackType f;

	  f = boost::bind(&RLSestimator::DynamicReconfigureCallback, this, _1, _2);
	  server.setCallback(f);

	  // subscribe to RVIZ
	  paramTuneSub_ = nh.subscribe("/tunedParams", 1000, &RLSestimator::ParamTuneCallback, this);
	}

        ~RLSestimator(){};
        
	//--------- For dynamic Reconfigure -------------

	void DynamicReconfigureCallback(ros_rls_estimation::rlsConfig &config, uint32_t level) {
	  
	  cuttingFailureThreshold = config.cuttingTh;
	  slidingThreshold = config.slidingTh;
	  contactThreshold = config.contactTh;
	  tearingThreshold = config.tearingTh;
	  FnobservatbilityThreshold = config.FnObsTh;
	  lamda_Fc = config.FcFF;
	  lamda_mu = config.muFF;
	  motionThreshold = config.motionTh;
	  windowSize = config.windowSizeFn;
	}

	//------------- For RVIZ paramTune-----------------------

	void ParamTuneCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
	{
	  cuttingFailureThreshold = (double)array->data.at(0)/100;
	  slidingThreshold = (double)array->data.at(1)/100;
	  tearingThreshold = (double)array->data.at(2)/100;
	  motionThreshold = (double)array->data.at(3)/1000;
	  FnobservatbilityThreshold = (double)array->data.at(4)/100;
	  lamda_Fc = (double)array->data.at(5)/100;
	  lamda_mu = (double)array->data.at(6)/100;
	  windowSize = array->data.at(7);

	//  std::cout<<"cth: "<< cuttingFailureThreshold<<std::endl;
	//  std::cout<<"sth: "<< slidingThreshold<<std::endl;
	//  std::cout<<"tth: "<< tearingThreshold<<std::endl;
	//  std::cout<<"mth: "<< motionThreshold<<std::endl;
	}
	//------------------------------------------------------


        //returns true if the cutter is in contact with the cutting surface
        bool inContact(const double& Fn){
	  if(abs(Fn) < contactThreshold && contactState == true){
                std::cout<< "Cutter in Contact" << std::endl;
                contactState = false;
            }
	  if(abs(Fn) > contactThreshold && contactState == false){
                std::cout<< "Cutter not in Contact!" << std::endl;
                contactState = true;
            }

	  if(abs(Fn) < contactThreshold){
                return false;
            }
	  if(abs(Fn) > contactThreshold){
                return true;
            }
        }

        // detects if WAM is in motion
        bool WAMnotMoving(const bool& wamNotMoving){
            if(wamNotMoving && wamMotionState == true){
                std::cout<< "WAM STOPPED!" << std::endl;
                wamMotionState = false;
                return true;
            }
            if(!wamNotMoving && wamMotionState == false){
               std::cout<< "WAM moving" << std::endl;
               wamMotionState = true;
                return false;
            }
        } 

               
       
        void GetEstimates (vctDoubleVec& rlsEstDataCopy) const{
            rlsEstDataCopy = rlsEstData;
        }        
         
        
	void EvaluateRecursiveLS( const double &Fn, 
				  const double &Fe, 
				  Eigen::Vector2d& xnew,
				  Eigen::Matrix2d& Pnew){		
	        double yk = Fe;
		Eigen::Vector2d Hk(copysign(Fn,1),1);

		Eigen::Vector2d K;
                
		K = Pnew*Hk/(Hk.transpose()*Pnew*Hk + 1);

                xnew = xnew + K*(yk - Hk.transpose()*xnew);

		Eigen::Matrix2d Lamda;
		Lamda(0,0) = 1/lamda_mu;
		Lamda(0,1) = 0;
		Lamda(1,0) = 0;
		Lamda(1,1) = 1/lamda_Fc;
		
		Pnew = Lamda*(Eigen::Matrix2d::Identity() - K*Hk.transpose())*Pnew*Lamda;
		  
               }

	Eigen::Vector2d EvaluateMovingWindowLS(const double& x,
					       const double& y,
					       std::vector<double>& xvec,
					       std::vector<double>& yvec,
					       int& windowSize){
	 
	  xvec.push_back(x);
	  yvec.push_back(y);
	 
	  Eigen::Vector2d params;
	  if(xvec.size() < 5){
	    params<<0 ,0;
	    return params;
	  }
	    
	  if(xvec.size() > windowSize){
	    xvec.erase(xvec.begin());
	    yvec.erase(yvec.begin());
	  }
	    

	    Eigen::MatrixXd X(xvec.size(),2);
	    Eigen::VectorXd Y(yvec.size());
	    for(int i = 0; i < xvec.size(); ++i)
	      for(int j = 0; j < 2; ++j){
		if(j = 0){X(i,j) = xvec.at(i);}
		if(j = 1){X(i,j) = 1;}
		
		Y(i) = yvec.at(i);
	      }
	      
	    params = (X.transpose()*X).inverse()*X.transpose()*Y;
	    
	    return params;
	 
	}


	bool DetectAnomaly( const double &Fn, const double &Fe, const double& Fest ){

                
		bool anomalyFail, anomalyTear, anomalySlip;
		// Fest > Fe + slidingThreshold indicate sliding without cutting
		if(Fn < 0 && Fest - Fe > slidingThreshold){
		  std::cout<< "sliding withoug cutting at time:" << currTime << std::endl;
		  haveSlided = 10;
		  anomalySlip = true;
		}
		else{
		  haveSlided = 0;
		  anomalySlip = false;
		}

		// catch tearing
		if(Fn > 0 && Fest - Fe > tearingThreshold){
		  std::cout << "tearing at time: " << currTime << std::endl;
		  haveTeared = 10;
		  anomalyTear = true;
		}
		else{
		  haveTeared = 0;
		  anomalyTear = false;
		}
		// Fe > Fest + cuttingFailureThreshold indicate cutting failure
		if((Fe - Fest > cuttingFailureThreshold && Fn < 0) || Fe > 15){
		  haveFailed = 10;
		  anomalyFail = true;
		}
		else{
		  haveFailed = 0;
		  anomalyFail = false;
		}

		if(anomalySlip || anomalyTear || anomalyFail){
		 
		  xlast = xLastSuccessful;
		  Plast = PlastSuccessful;
		  return true;
		}
		else{
		  xLastSuccessful = xlast;
		  PlastSuccessful = Plast;
		  return false;
		}

        }

       void ZeroFailures(){
	 haveSlided = 0;
	 haveTeared = 0;
	 haveFailed = 0;
       }

       bool EvaluateEstimator(const double& Fn, 
			      const double& Fe,
			      std::string mode){

	 if(mode == "Adaptive"){
	   x = xlast;
	   P = Plast;
	   
	   EvaluateRecursiveLS(Fn, Fe, x, P);
	 
	   // if the estimated result is physically unreasonable, discard and evaluate statically
	   if(x(0) < 0 || x(1) < 0){
	     x = xlast;
	     P = Plast;
	     }
	   
	   xlast = x;
	   Plast = P;
	   Fest = x(0)*copysign(Fn,1) + x(1);
	 }

	 if(mode == "Static"){
	   x = xlast;
	   Fest = x(0)*copysign(Fn,1) + x(1);
	 }
       
	 return DetectAnomaly(Fn, Fe, Fest); 
	 
       }

      
   void RLSestimate(const double &Fn, 
                    const double &Fe, 
                    const double& currtime,
                    const vctDynamicVector<double>& currJointPos){

        
     currTime = currtime;
     wamNotMoving = WAMIsNotMoving(currJointPos, currtime);
     WAMnotMoving(wamNotMoving);
     Eigen::Vector2d FnX = EvaluateMovingWindowLS(currTime, Fn, timeVec, FnVec, windowSize);    
           
       bool anomaly;
       
       //if(wamNotMoving || !inContact(Fn) || Fe < 0){
	 if(wamNotMoving || Fe < 0){
	   ZeroFailures();
	   x<<0,0;
	   Fest = 0;
	   rlsEstData.Assign((double)Fe, Fn, x(0), x(1), Fest, FnX(0), haveFailed, haveTeared, haveSlided);	      
       }	 
       else{


	 if(FnX(0) < FnobservatbilityThreshold){
	   anomaly = EvaluateEstimator(Fn, Fe, "Static");
	 }
	 else{
	   anomaly = EvaluateEstimator(Fn, Fe, "Adaptive");
	   
	   }
  
	 if(anomaly){
	     x << 0,0;
	     Fest = 0;
	   }
	 else{ZeroFailures();}

	 rlsEstData.Assign((double)Fe, Fn, x(0), x(1), Fest, FnX(0), haveFailed, haveTeared, haveSlided);
       }
   }




//----------------------------------  OTHER ---------------------------------------------------------------


  void CalcVectorAverage(const std::vector<double>& vect, double& avg){

      double temp = 0;
      for(int i = 0; i < vect.size(); i++){
            temp = temp + vect.at(i);
      }

      avg = temp/vect.size();
  }

void CalcAverageVelocity(const vctDynamicVector<double>& currJointPos, const double& currTime, vctDynamicVector<double>& avgVel){

    std::vector<double> temp;

    for(int i = 0; i < 7; i++){
        temp.push_back(currJointPos[i]);
    }

         jointPoses.push_back(temp);
         timeStamps.push_back(currTime);
        // process only the most recent avgNum data
        if(timeStamps.size() > avgNum){
            jointPoses.erase(jointPoses.begin());
            timeStamps.erase(timeStamps.begin());
        }
        vctDynamicVector<double> currPos( 7 , jointPoses.at(avgNum-1).at(0),jointPoses.at(avgNum-1).at(1),jointPoses.at(avgNum-1).at(2),jointPoses.at(avgNum-1).at(3),jointPoses.at(avgNum-1).at(4),jointPoses.at(avgNum-1).at(5),jointPoses.at(avgNum-1).at(6));

        vctDynamicVector<double> pastPos( 7 , jointPoses.at(0).at(0),jointPoses.at(0).at(1),jointPoses.at(0).at(2),jointPoses.at(0).at(3),jointPoses.at(0).at(4),jointPoses.at(0).at(5),jointPoses.at(0).at(6));

	if(timeStamps.size() >= avgNum){
	  vctDynamicVector<double> jointPosdiff = (currPos - pastPos).Abs();
	  double timediff = (timeStamps.at(avgNum-1) - timeStamps.at(0));
	  avgVel = jointPosdiff / timediff;
	  
	}
        else{
	  avgVel = currPos;
	}
         
}

bool WAMIsNotMoving( const vctDynamicVector<double>& currJointPos, const double& currTime){
      
    vctDynamicVector<double> jointAvgVel;
    CalcAverageVelocity(currJointPos, currTime, jointAvgVel);
    avgJointVel = jointAvgVel.MaxElement();

        if(jointAvgVel[0] < motionThreshold && jointAvgVel[1] < motionThreshold && jointAvgVel[2] < motionThreshold && jointAvgVel[3] < motionThreshold && jointAvgVel[4] < motionThreshold && jointAvgVel[5] < motionThreshold && jointAvgVel[6] < motionThreshold){

            //std::cout<<jointAvgVel<<std::endl;
            return true;
        }
        else{

            //std::cout<<jointAvgVel<<std::endl;
            return false;
        }
}

void Inverse(vctFixedSizeMatrix<double,2,2>& M, vctFixedSizeMatrix<double,2,2>& Minv){
            double detM;
            detM = M[0][0]*M[1][1] - M[0][1]*M[1][0];
            
            if(abs(detM) < 0.0001){
                detM = 0.0001;
            }

            Minv[0][0] = M[1][1];
            Minv[0][1] = -M[0][1];
            Minv[1][0] = -M[1][0];
            Minv[1][1] = M[0][0];

            Minv = (1/detM)*Minv;


        }

};




#endif

