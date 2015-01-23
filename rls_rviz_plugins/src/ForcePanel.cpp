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

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QProgressBar>
#include <cmath>
//#include <geometry_msgs/Twist.h>

//#include "drive_widget.h"
#include "ForcePanel.h"

namespace rls_rviz_plugins
{


ForcePanel::ForcePanel( QWidget* parent )
  : rviz::Panel( parent )
  , Ft_( 0 )
  , Fn_( 0 )
{
  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* topic_layout = new QHBoxLayout;
  Ft_topic_editor_ = new QLineEdit;
  Fn_topic_editor_ = new QLineEdit; 

 //topic_layout->addWidget( new QLabel( "Output Topic:" ));
  topic_layout->addWidget( new QLabel( "Normal Force Topic:" ) );
  topic_layout->addWidget( Fn_topic_editor_ ); 

  topic_layout->addWidget( new QLabel( "Resistive Force Topic:" ) );
  topic_layout->addWidget( Ft_topic_editor_ );
  

  QHBoxLayout* progressbar_layout = new QHBoxLayout;

  Ft_progress_bar_ = new QProgressBar;
  Fn_progress_bar_ = new QProgressBar;
  
  Fn_progress_bar_->setRange( 0, 400 );
  Ft_progress_bar_->setRange( 0, 400 );
  Fn_progress_bar_->setTextVisible( false );
  Ft_progress_bar_->setTextVisible( false );
  
  progressbar_layout->addWidget( new QLabel( "-20(N)" ) );
  progressbar_layout->addWidget( Fn_progress_bar_ );
  progressbar_layout->addWidget( new QLabel( "20(N)     " ) );
  progressbar_layout->addWidget( new QLabel( "-20(N)" ) );
  progressbar_layout->addWidget( Ft_progress_bar_ );
  progressbar_layout->addWidget( new QLabel( "20(N)" ) );

  QHBoxLayout* showForces_layout = new QHBoxLayout;
  Fn_label_ = new QLabel;
  Ft_label_ = new QLabel;
  
  showForces_layout->addWidget( new QLabel( "" ) );
  showForces_layout->addWidget( Fn_label_ );
  showForces_layout->addWidget( new QLabel( "(N)" ) );
  showForces_layout->addWidget( new QLabel( "" ) );
  showForces_layout->addWidget( Ft_label_ );
  showForces_layout->addWidget( new QLabel( "(N)" ) );
  

//output_topic_editor_ = new QLineEdit;
  //topic_layout->addWidget( output_topic_editor_ );
 
  // Then create the control widget.
  //drive_widget_ = new DriveWidget;

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( topic_layout );
  layout->addLayout( progressbar_layout );
  layout->addLayout( showForces_layout );
  // layout->addWidget( drive_widget_ );
  setLayout( layout );

  QTimer* output_timer = new QTimer( this );

  // Next we make signal/slot connections.
  //connect( drive_widget_, SIGNAL( outputVelocity( float, float )), this, SLOT( setVel( float, float )));
  connect( Fn_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopicFn() ));
  connect( Ft_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopicFt() ));
  //connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));
  
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( updateFnProgressBar() ));
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( updateFtProgressBar() ));
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( updateFnLabel() ));
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( updateFtLabel() ));
  
  // Start the timer.
  output_timer->start( 10 );

  // Make the control widget start disabled, since we don't start with an output topic.
  // drive_widget_->setEnabled( false );
}

  void ForcePanel::updateFnLabel(){
    Fn_label_->setNum(Fn_);
  }

 void ForcePanel::updateFtLabel(){
    Ft_label_->setNum(Ft_);
  }

  void ForcePanel::updateFnProgressBar(){
    Fn_progress_bar_->setValue(mapFloatForceToInt(Fn_));
  }

 void ForcePanel::updateFtProgressBar(){
    Ft_progress_bar_->setValue(mapFloatForceToInt(Ft_));
  }

  int ForcePanel::mapFloatForceToInt(const float& F){
    return round( ( F + 20 )*10 );
  }

  void ForcePanel::updateTopicFn()
  {
    setTopicFn( Fn_topic_editor_->text() );
  }
  
  void ForcePanel::updateTopicFt()
  {
    setTopicFt( Ft_topic_editor_->text() );
  }
  
  
  // Set the topic name we are publishing to.
  void ForcePanel::setTopicFn( const QString& Fn_topic )
  {
    // Only take action if the name has changed.
    if( Fn_topic != Fn_topic_)
      {
	Fn_topic_ = Fn_topic;
	// If the topic is the empty string, don't publish anything.
	if( Fn_topic_ == "")
	  {
	    FnSub_.shutdown();
	  }
	else
	  {
	    FnSub_ = nh_.subscribe<std_msgs::Float64>( Fn_topic_.toStdString(), 1000, &ForcePanel::FnCallback, this );
    }
	// rviz::Panel defines the configChanged() signal.  Emitting it
	// tells RViz that something in this panel has changed that will
	// affect a saved config file.  Ultimately this signal can cause
	// QWidget::setWindowModified(true) to be called on the top-level
	// rviz::VisualizationFrame, which causes a little asterisk ("*")
	// to show in the window's title bar indicating unsaved changes.
	Q_EMIT configChanged();
      }

  // Gray out the control widget when the output topic is empty.
  //drive_widget_->setEnabled( output_topic_ != "" );
}

  void ForcePanel::setTopicFt( const QString& Ft_topic )
{
  // Only take action if the name has changed.
  if( Ft_topic != Ft_topic_)
  {
    Ft_topic_ = Ft_topic;
    // If the topic is the empty string, don't publish anything.
    if( Ft_topic_ == "")
    {
      FtSub_.shutdown();
    }
    else
    {
       FtSub_ = nh_.subscribe<std_msgs::Float64>( Ft_topic_.toStdString(), 1000, &ForcePanel::FtCallback, this );
    }
    
    Q_EMIT configChanged();
  }

  // Gray out the control widget when the output topic is empty.
  //drive_widget_->setEnabled( output_topic_ != "" );
}

  
// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void ForcePanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "FnTopic", Fn_topic_ );
  config.mapSetValue( "FtTopic", Ft_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void ForcePanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString Fntopic;
  QString Fttopic;
  if( config.mapGetString( "FnTopic", &Fntopic ) && config.mapGetString( "FtTopic", &Fttopic ) )
  {
    Fn_topic_editor_->setText( Fntopic );
    Ft_topic_editor_->setText( Fttopic );
    updateTopicFn();
    updateTopicFt();
  }
}

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rls_rviz_plugins::ForcePanel,rviz::Panel )
// END_TUTORIAL
