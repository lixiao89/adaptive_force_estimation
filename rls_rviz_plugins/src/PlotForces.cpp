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

#include <iostream>

#include "PlotForces.h"
#include "ForceDrawWidget.h"
#include <QTimer>
#include <QVBoxLayout>

namespace rls_rviz_plugins
{


PlotForces::PlotForces( QWidget* parent )
  : rviz::Panel( parent )
  , Ft( 0 )
  , Fest( 0 )
  , Fn( 0 )
{
  
  QTimer* dataTimer = new QTimer;
  w = new QWidget;
  cp = new QCustomPlot(w);
  
  QVBoxLayout* layout = new QVBoxLayout;
  
  layout->addWidget(cp);
  setLayout(layout);

  setupPlot();

  rlsSub_=nh_.subscribe<cisst_msgs::rlsEstData>( "rlsEstData", 1000, &PlotForces::getrlsEstDataCallback, this );
 
  connect(dataTimer, SIGNAL( timeout() ), this, SLOT( Plot() ) );
  dataTimer->start(10); // Interval 0 means to refresh as fast as possible          
    

}


  void PlotForces::setupPlot(){

     cp->plotLayout()->clear();
    
    topAxisRect = new QCPAxisRect( cp );
    bottomAxisRect = new QCPAxisRect( cp );
    
    cp->plotLayout()->addElement(0, 0, topAxisRect);
    cp->plotLayout()->addElement(1, 0, bottomAxisRect);
   
    Ftgraph = new QCPGraph(topAxisRect->axis(QCPAxis::atBottom), topAxisRect->axis(QCPAxis::atLeft));
    Festgraph = new QCPGraph(topAxisRect->axis(QCPAxis::atBottom), topAxisRect->axis(QCPAxis::atLeft));
 

    Fngraph = new QCPGraph(bottomAxisRect->axis(QCPAxis::atBottom), bottomAxisRect->axis(QCPAxis::atLeft));

    

    Ftgraph = cp->addGraph(topAxisRect->axis(QCPAxis::atBottom), topAxisRect->axis(QCPAxis::atLeft));
    Ftgraph->setName("Measured Resist. Force");
    Ftgraph->setPen(QPen(Qt::blue));
    Ftgraph->setBrush(QBrush(QColor(240, 255, 200)));
    Ftgraph->setAntialiasedFill(false);

    Festgraph = cp->addGraph(topAxisRect->axis(QCPAxis::atBottom), topAxisRect->axis(QCPAxis::atLeft));
    Festgraph->setName("Estimated Resist. Force");
    Festgraph->setPen(QPen(Qt::red));
    Festgraph->setBrush(QBrush(QColor(220, 255, 200, 128)));
    Festgraph->setAntialiasedFill(false);

   
    Fngraph = cp->addGraph(bottomAxisRect->axis(QCPAxis::atBottom), bottomAxisRect->axis(QCPAxis::atLeft));
    Fngraph->setName("Measured Normal Force");
    Fngraph->setPen(QPen(Qt::red));
    Fngraph->setBrush(QBrush(QColor(220, 255, 200, 128)));
    Fngraph->setAntialiasedFill(false);


    // setup axis
    topAxisRect->axis(QCPAxis::atBottom)->setTickLabelType(QCPAxis::ltDateTime);
    topAxisRect->axis(QCPAxis::atBottom)->setDateTimeFormat("hh:mm:ss");
    topAxisRect->axis(QCPAxis::atBottom)->setAutoTickStep(false);
    topAxisRect->axis(QCPAxis::atBottom)->setTickStep(2);
    topAxisRect->setupFullAxesBox(true);
   
    bottomAxisRect->axis(QCPAxis::atBottom)->setTickLabelType(QCPAxis::ltDateTime);
    bottomAxisRect->axis(QCPAxis::atBottom)->setDateTimeFormat("hh:mm:ss");
    bottomAxisRect->axis(QCPAxis::atBottom)->setAutoTickStep(false);
    bottomAxisRect->axis(QCPAxis::atBottom)->setTickStep(2);
    bottomAxisRect->setupFullAxesBox(true);
    

    //set axis labels, titles, legends
    Ftgraph->addToLegend();
    Festgraph->addToLegend();
    topAxisRect->axis(QCPAxis::atBottom)->setLabel("time");
    topAxisRect->axis(QCPAxis::atLeft)->setLabel("Force (N)");

    Fngraph->addToLegend();
    bottomAxisRect->axis(QCPAxis::atBottom)->setLabel("time");
    bottomAxisRect->axis(QCPAxis::atLeft)->setLabel("Force (N)");

    // setup an extra legend for that axis rect:
    QCPLegend *topLegend = new QCPLegend;
    topAxisRect->insetLayout()->addElement(topLegend, Qt::AlignTop|Qt::AlignLeft);
    topLegend->setLayer("legend");
    cp->setAutoAddPlottableToLegend(false); // would add to the main legend (in the primary axis rect)
    // add a legend item to the new legend, representing the graph:
    topLegend->addItem(new QCPPlottableLegendItem(topLegend, Ftgraph));
    topLegend->addItem(new QCPPlottableLegendItem(topLegend, Festgraph));
    
    QCPLegend *bottomLegend = new QCPLegend;
    bottomAxisRect->insetLayout()->addElement(bottomLegend, Qt::AlignTop|Qt::AlignLeft);
    bottomLegend->setLayer("legend");
    cp->setAutoAddPlottableToLegend(false); // would add to the main legend (in the primary axis rect)
    // add a legend item to the new legend, representing the graph:
    bottomLegend->addItem(new QCPPlottableLegendItem(bottomLegend, Fngraph));
     

    // since we've created the axis rects and axes from scratch, we need to place them on
    // according layers, if we don't want the grid to be drawn above the axes etc.
    // place the axis on "axes" layer and grids on the "grid" layer, which is below "axes":

    QList<QCPAxis*> allAxes;
    allAxes << topAxisRect->axes() << bottomAxisRect->axes();
    foreach (QCPAxis *axis, allAxes)
      {
	axis->setLayer("axes");
	axis->grid()->setLayer("grid");
      }

    // make left and bottom axes transfer their ranges to right and top axes\       

     connect(topAxisRect->axis(QCPAxis::atBottom), SIGNAL(rangeChanged(QCPRange)), topAxisRect->axis(QCPAxis::atBottom), SLOT(setRange(QCPRange)));
     connect(topAxisRect->axis(QCPAxis::atLeft), SIGNAL(rangeChanged(QCPRange)), topAxisRect->axis(QCPAxis::atLeft),SLOT(setRange(QCPRange)));

     connect(bottomAxisRect->axis(QCPAxis::atBottom), SIGNAL(rangeChanged(QCPRange)), bottomAxisRect->axis(QCPAxis::atBottom), SLOT(setRange(QCPRange)));
     connect(bottomAxisRect->axis(QCPAxis::atLeft), SIGNAL(rangeChanged(QCPRange)), bottomAxisRect->axis(QCPAxis::atLeft),SLOT(setRange(QCPRange)));


  }

  void PlotForces::Plot(){
 
     double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;

     static double lastPointKey = 0;
     if( key - lastPointKey > 0.01 ){
    //   // add data to lines:                                                         
       Ftgraph->addData(key, Ft);
       Festgraph->addData(key, Fest);
       Fngraph->addData(key, Fn);
     
       Ftgraph->removeDataBefore(key-8);
       Festgraph->removeDataBefore(key-8);
       Fngraph->removeDataBefore(key-8);
     
       Ftgraph->rescaleValueAxis(true);
       Festgraph->rescaleValueAxis(true);
       Fngraph->rescaleValueAxis(true);
       lastPointKey = key;
     }


     topAxisRect->axis(QCPAxis::atBottom)->setRange(key+0.25, 8, Qt::AlignRight);
     bottomAxisRect->axis(QCPAxis::atBottom)->setRange(key+0.25, 8, Qt::AlignRight);
     cp->replot();
  }


// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void PlotForces::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

// Load all configuration data for this panel from the given Config object.
void PlotForces::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
}

} 

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rls_rviz_plugins::PlotForces,rviz::Panel )

