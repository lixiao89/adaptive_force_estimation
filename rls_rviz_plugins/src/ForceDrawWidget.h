#ifndef FORCE_DRAW_WIDGET_H
#define FORCE_DRAW_WIDGET_H


#include <ros/ros.h>

#include <QWidget>
#include <QTimer>
#include "qcustomplot/qcustomplot.h"
#include <cisst_msgs/rlsEstData.h>


namespace rls_rviz_plugins{

class ForceDrawWidget : public QWidget
{
     Q_OBJECT

public:
  ForceDrawWidget(QWidget *parent = 0);

  // override painEvent to draw graph
  //virtual void paintEvent( QPaintEvent* event );

  void Plot( QCustomPlot* customPlot );

   public Q_SLOTS:
    void realtimeDataSlot();
    void setForceValues( double Ft_, double Fest_, double Ferror_ );
    //protected:
   
 public:
  
  QCustomPlot* customPlot;
  double Ft, Fest, Ferror;
  QTimer dataTimer;
  //ros::Subscriber rlsSub_;
  //ros::NodeHandle nh_;
};

}
 #endif
