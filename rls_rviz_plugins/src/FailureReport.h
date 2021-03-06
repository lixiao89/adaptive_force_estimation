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
#ifndef FAILURE_REPORT_H
#define FAILURE_REPORT_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

#include <rviz/panel.h>
#endif

#include <std_msgs/Float64.h>
#include <cisst_msgs/rlsEstData.h>

#include "ForcePanel.h"

class QLabel;
class QPixmap;
class QPushButton;

namespace rls_rviz_plugins
{

class FailureReport: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
 
  FailureReport( QWidget* parent = 0 );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

protected Q_SLOTS:

  void updateLabel();
  void showNewPanel();
 protected:

  void FailureReportCallback(const cisst_msgs::rlsEstData::ConstPtr& rlsEst_msg){
    havebunched_ = rlsEst_msg->haveFailed;
    haveteared_ = rlsEst_msg->haveTeared;
    haveslipped_ = rlsEst_msg->haveSlided;
  } 


  QPixmap* normalPix_;
  QPixmap* bunchPix_;
  QPixmap* tearPix_;
  QPixmap* slipPix_;
  
  QLabel* failureReport_label_;

  ros::Subscriber rlsSub_;
     
  // The ROS node handle.
  ros::NodeHandle nh_;

  // used to store data
  double havebunched_;
  double haveteared_;
  double haveslipped_;


};

} // end namespace rviz_plugin_tutorials

#endif // TELEOP_PANEL_H
