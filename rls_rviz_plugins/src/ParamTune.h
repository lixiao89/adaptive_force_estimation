/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef PARAM_TUNE_H
#define PARAM_TUNE_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

class QSlider;

namespace rls_rviz_plugins
{

class ParamTune: public rviz::Panel
{

Q_OBJECT
public:

  ParamTune( QWidget* parent = 0 );


  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

 
protected Q_SLOTS:

  void updateParams();
  void publishParams();

protected:

  QSlider* cuttingTh;
  QSlider* slidingTh;
  QSlider* tearingTh;
  QSlider* motionTh;
  QSlider* FnobsTh;
  QSlider* lambda_mu;
  QSlider* lambda_Fc;
  QSlider* windowSize;

  
  QString output_topic_;


  ros::Publisher param_publisher_;


  ros::NodeHandle nh_;

  //params =[ cuttingTh,
  //          slidingTh,
  //          tearingTh,
  //          motionTh ]
 

  //          FnObs,
  //          lambda_Fc,
  //          lambda_mu,
  //          contactTh,
  //          windowSize 
  std_msgs::Int32MultiArray params;
 
};

}

#endif 