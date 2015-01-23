
 #include <QtGui>
 #include "ForceDrawWidget.h"
#include <iostream>


namespace rls_rviz_plugins{

 ForceDrawWidget::ForceDrawWidget(QWidget *parent)
     : QWidget(parent)
     , Ft( 0 )
     , Fest( 0 )
     , Ferror( 0 )
 {
   //   QTimer *timer = new QTimer(this);
//connect(timer, SIGNAL(timeout()), this, SLOT(update()));
//     timer->start(0);
   setWindowTitle(tr("Forces"));
   resize(200, 200);
   customPlot = new QCustomPlot(this);
  
     //     Plot( customPlot );
     
     // rlsSub_ = nh_.subscribe<cisst_msgs::rlsEstData>( "rlsEstData", 1000, &PlotForces::getrlsEstDataCallback, this );

 }


void ForceDrawWidget::setForceValues( double Ft_, double Fest_, double Ferror_ ){
  Ft = Ft_;
  Fest = Fest_;
  Ferror = Ferror_;
  std::cout<<"HERE!"<<std::endl;
}

void ForceDrawWidget::Plot( QCustomPlot *customPlot ){
  customPlot->addGraph();
  customPlot->graph(0)->setPen(QPen(Qt::blue));
  customPlot->graph(0)->setBrush(QBrush(QColor(240, 255, 200)));
  customPlot->graph(0)->setAntialiasedFill(false);

  customPlot->xAxis->setTickLabelType(QCPAxis::ltDateTime);
  customPlot->xAxis->setDateTimeFormat("hh:mm:ss");
  customPlot->xAxis->setAutoTickStep(false);
  customPlot->xAxis->setTickStep(2);
  customPlot->axisRect()->setupFullAxesBox();

  // make left and bottom axes transfer their ranges to right and top axes\
       
  connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
  connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2, SLOT(setRange(QCPRange)));

  // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:     
  connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
  dataTimer.start(0); // Interval 0 means to refresh as fast as possible  

}

void ForceDrawWidget::realtimeDataSlot(){

  double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;

  static double lastPointKey = 0;
  if( key - lastPointKey > 0.01 ){
    // add data to lines:                                                  
    customPlot->graph(0)->addData(key, Ft);
    customPlot->graph(1)->addData(key, Fest);
    // set data of dots:                                                   
    // ui->customPlot->graph(2)->clearData();                              
    // ui->customPlot->graph(2)->addData(key, value0);                     
    // ui->customPlot->graph(3)->clearData();                              
    // ui->customPlot->graph(3)->addData(key, value1);                     
    // remove data of lines that's outside visible range:                  
    customPlot->graph(0)->removeDataBefore(key-8);
    customPlot->graph(1)->removeDataBefore(key-8);
    // rescale value (vertical) axis to fit the current data:              
    customPlot->graph(0)->rescaleValueAxis();
    customPlot->graph(1)->rescaleValueAxis(true);
    lastPointKey = key;
  }

 
  customPlot->xAxis->setRange(key+0.25, 8, Qt::AlignRight);
  customPlot->replot();


}

  }
