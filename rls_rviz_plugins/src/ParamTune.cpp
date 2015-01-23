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
#include <QLabel>
#include <QTimer>
#include <QSlider>
#include "ParamTune.h"

namespace rls_rviz_plugins
{


ParamTune::ParamTune( QWidget* parent )
  : rviz::Panel( parent )
{
  
  params.data.push_back(200);
  params.data.push_back(180);
  params.data.push_back(180);
  params.data.push_back(1);
  params.data.push_back(30);
  params.data.push_back(97);
  params.data.push_back(99);
  params.data.push_back(80);

  cuttingTh = new QSlider(Qt::Horizontal);
  slidingTh = new QSlider(Qt::Horizontal);
  tearingTh = new QSlider(Qt::Horizontal);
  motionTh = new QSlider(Qt::Horizontal);
  FnobsTh = new QSlider(Qt::Horizontal);
  lambda_mu = new QSlider(Qt::Horizontal);
  lambda_Fc = new QSlider(Qt::Horizontal);
  windowSize = new QSlider(Qt::Horizontal);

  cuttingTh->setRange( 0, 500 );
  slidingTh->setRange( 0, 500 );
  tearingTh->setRange( 0, 500 );
  motionTh->setRange( 0, 100 );
  FnobsTh->setRange( 0, 50 );
  lambda_mu->setRange( 0, 100 );
  lambda_Fc->setRange( 0, 100 );
  windowSize->setRange( 0, 200);


  cuttingTh->setValue(params.data.at(0));
  slidingTh->setValue(params.data.at(1));
  tearingTh->setValue(params.data.at(2));
  motionTh->setValue(params.data.at(3));
  FnobsTh->setValue(params.data.at(4));
  lambda_mu->setValue(params.data.at(5));
  lambda_Fc->setValue(params.data.at(6));
  windowSize->setValue(params.data.at(7));

  QHBoxLayout* overallLayout = new QHBoxLayout;

  QVBoxLayout* leftlayout = new QVBoxLayout;
   
  leftlayout->addWidget( new QLabel("Bunching Threshold") );
  leftlayout->addWidget( cuttingTh );
  leftlayout->addWidget( new QLabel("Sliding Threshold") );
  leftlayout->addWidget( slidingTh );
  leftlayout->addWidget( new QLabel("Tearing Threshold") );
  leftlayout->addWidget( tearingTh );
  leftlayout->addWidget( new QLabel("Motion Threshold") );
  leftlayout->addWidget( motionTh );

  QVBoxLayout* rightlayout = new QVBoxLayout;
   
  rightlayout->addWidget( new QLabel("Fn Observability Threshold") );
  rightlayout->addWidget( FnobsTh );
  rightlayout->addWidget( new QLabel("mu Forgetting Factor") );
  rightlayout->addWidget( lambda_mu );
  rightlayout->addWidget( new QLabel("Fc Forgetting Factor") );
  rightlayout->addWidget( lambda_Fc );
  rightlayout->addWidget( new QLabel("Fn LS Window Size") );
  rightlayout->addWidget( windowSize );

  overallLayout->addLayout(leftlayout);
  overallLayout->addLayout(rightlayout);
  setLayout( overallLayout );


  QTimer* output_timer = new QTimer( this );
  
  connect( cuttingTh, SIGNAL( valueChanged(int) ), this, SLOT( updateParams() ));
  connect( slidingTh, SIGNAL( valueChanged(int) ), this, SLOT( updateParams() ));
  connect( tearingTh, SIGNAL( valueChanged(int) ), this, SLOT( updateParams() ));
  connect( motionTh, SIGNAL( valueChanged(int) ), this, SLOT( updateParams() ));   
  connect( FnobsTh, SIGNAL( valueChanged(int) ), this, SLOT( updateParams() ));   
  connect( lambda_mu, SIGNAL( valueChanged(int) ), this, SLOT( updateParams() ));   
  connect( lambda_Fc, SIGNAL( valueChanged(int) ), this, SLOT( updateParams() ));  
  connect( windowSize, SIGNAL( valueChanged(int) ), this, SLOT( updateParams() ));   
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( publishParams() ));

  // Start the timer.
  output_timer->start( 10 );

  param_publisher_ = nh_.advertise<std_msgs::Int32MultiArray>( "/tunedParams", 1000 );
}


  void ParamTune::updateParams(){
    params.data.clear();
    params.data.push_back(cuttingTh->value());
    params.data.push_back(slidingTh->value());
    params.data.push_back(tearingTh->value());
    params.data.push_back(motionTh->value());
    params.data.push_back(FnobsTh->value());
    params.data.push_back(lambda_mu->value());
    params.data.push_back(lambda_Fc->value());
    params.data.push_back(windowSize->value());
    Q_EMIT configChanged();
  }

  void ParamTune::publishParams(){
    param_publisher_.publish(params);
  } 

void ParamTune::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  //config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void ParamTune::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  //QString topic;
  //if( config.mapGetString( "Topic", &topic ))
  //{
  //  output_topic_editor_->setText( topic );
  //  updateTopic();
  //}
}

} 

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rls_rviz_plugins::ParamTune,rviz::Panel )
// END_TUTORIAL
