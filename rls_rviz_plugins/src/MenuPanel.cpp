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

#include <stdio.h>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QPushButton>

#include "MenuPanel.h"
#include "ForcePanel.h"
#include "FailureReport.h"
#include "ParamTune.h"
#include "PlotForces.h"
#include "ParamTuneViz.h"
#include "PlotEstParams.h"

namespace rls_rviz_plugins
{


MenuPanel::MenuPanel( QWidget* parent )
  : rviz::Panel( parent )
 {

  forcepanel = new ForcePanel();
  failurereport = new FailureReport();
  paramtune = new ParamTune();
  plotforces = new PlotForces();
  paramtuneviz = new ParamTuneViz();
  plotestparams = new PlotEstParams();


  forcepanelbutton = new QPushButton( "Forces" );
  failurereportbutton = new QPushButton( "Failure Report" );
  paramtunebutton = new QPushButton( "Tune" );
  plotforcesbutton = new QPushButton( "Plot Measurements" );
  paramtunevizbutton = new QPushButton( "Visualize Tuning" );
  plotestparamsbutton = new QPushButton( "Plot Estimations" );
  
  QHBoxLayout* layout = new QHBoxLayout;
  
  layout->addWidget( forcepanelbutton );
  layout->addWidget( failurereportbutton );
  layout->addWidget( paramtunebutton );
  layout->addWidget( plotforcesbutton );
  layout->addWidget( plotestparamsbutton );
  layout->addWidget( paramtunevizbutton );
  
  setLayout( layout );

  connect( forcepanelbutton, SIGNAL( clicked(bool) ), this, SLOT( showForcePanel() ));
  connect( failurereportbutton, SIGNAL( clicked(bool) ), this, SLOT( showFailureReport() ));
  connect( paramtunebutton, SIGNAL( clicked(bool) ), this, SLOT( showParamTune() ));
  connect( plotforcesbutton, SIGNAL( clicked(bool) ), this, SLOT( showPlotForces() ));
  connect( paramtunevizbutton, SIGNAL( clicked(bool) ), this, SLOT( showParamTuneViz() ));
  connect( plotestparamsbutton, SIGNAL( clicked(bool) ), this, SLOT( showPlotEstParams() ));

  

 }
 

  void MenuPanel::showForcePanel(){
     forcepanel->show();
  }

  void MenuPanel::showFailureReport(){
    failurereport->show();
  }

  void MenuPanel::showParamTune(){
    paramtune->show();
  }

  void MenuPanel::showPlotForces(){
    plotforces->show();
  }
  
  void MenuPanel::showParamTuneViz(){
    paramtuneviz->show();
  }

  void MenuPanel::showPlotEstParams(){
    plotestparams->show();
  }

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void MenuPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

// Load all configuration data for this panel from the given Config object.
void MenuPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
}

} 

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rls_rviz_plugins::MenuPanel,rviz::Panel )

