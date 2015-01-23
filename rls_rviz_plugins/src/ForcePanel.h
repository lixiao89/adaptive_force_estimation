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
#ifndef Force_PANEL_H
#define Force_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

#include <std_msgs/Float64.h>

class QLineEdit;
class QProgressBar;
class QLabel;

namespace rls_rviz_plugins
{


class ForcePanel: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:

  ForcePanel( QWidget* parent = 0 );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  // Next come a couple of public Qt slots.
public Q_SLOTS:

  void setTopicFn( const QString& Fn_topic );
  void setTopicFt( const QString& Ft_topic );
  // Here we declare some internal slots.
protected Q_SLOTS:

  void updateTopicFn();
  void updateTopicFt();

  void updateFnProgressBar();
  void updateFtProgressBar();


  void updateFnLabel();
  void updateFtLabel();
  // Then we finish up with protected member variables.
protected:

  void FnCallback(const std_msgs::Float64::ConstPtr& Fn_msg){
    Fn_ = Fn_msg->data;
  }

 void FtCallback(const std_msgs::Float64::ConstPtr& Ft_msg){
    Ft_ = Ft_msg->data;
} 

 int mapFloatForceToInt(const float& F);

  // The control-area widget which turns mouse events into command
  // velocities.
  //DriveWidget* drive_widget_;

  // One-line text editor for entering the outgoing ROS topic name.
  QLineEdit* Ft_topic_editor_;
  QLineEdit* Fn_topic_editor_;

  QProgressBar* Ft_progress_bar_;
  QProgressBar* Fn_progress_bar_;
  // The current name of the output topic.
  QString Ft_topic_;
  QString Fn_topic_;
  
  QLabel* Fn_label_;
  QLabel* Ft_label_;
  // The ROS publisher for the command velocity.
  //ros::Publisher velocity_publisher_;
  ros::Subscriber FtSub_;
  ros::Subscriber FnSub_;
    
  // The ROS node handle.
  ros::NodeHandle nh_;

  // used to store data
  float Fn_;
  float Ft_;


};

} // end namespace rviz_plugin_tutorials

#endif // TELEOP_PANEL_H
