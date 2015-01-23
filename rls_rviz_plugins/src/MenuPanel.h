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
#ifndef MENU_PANEL_H
#define MENU_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

#include <rviz/panel.h>
#include <rviz/panel_dock_widget.h>
#endif

#include <std_msgs/Float64.h>
#include <cisst_msgs/rlsEstData.h>



class QPushButton;


namespace rls_rviz_plugins
{
  
  class ForcePanel;
  class FailureReport;
  class ParamTune;
  class PlotForces;
  class ParamTuneViz;
  class PlotEstParams;

 class MenuPanel: public rviz::Panel
 {
   // This class uses Qt slots and is a subclass of QObject, so it needs
   // the Q_OBJECT macro.
Q_OBJECT
 public:
 
 MenuPanel( QWidget* parent = 0 );
 
 virtual void load( const rviz::Config& config );
 virtual void save( rviz::Config config ) const;
 
 protected Q_SLOTS:
   
   void showForcePanel();
   void showFailureReport();
   void showParamTune();
   void showPlotForces();
   void showParamTuneViz();
   void showPlotEstParams();
   
 protected:
   
   QPushButton* forcepanelbutton;
   QPushButton* failurereportbutton;
   QPushButton* paramtunebutton;
   QPushButton* plotforcesbutton;
   QPushButton* paramtunevizbutton;
   QPushButton* plotestparamsbutton;

   ForcePanel* forcepanel;
   FailureReport* failurereport;
   ParamTune* paramtune;
   PlotForces* plotforces;
   ParamTuneViz* paramtuneviz;
   PlotEstParams* plotestparams;
   
   
 };
 
} // end namespace rviz_plugin_tutorials

#endif // TELEOP_PANEL_H
