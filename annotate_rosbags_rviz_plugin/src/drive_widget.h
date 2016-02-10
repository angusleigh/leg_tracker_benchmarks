#ifndef DRIVE_WIDGET_H
#define DRIVE_WIDGET_H

#include <QWidget>

namespace annotate_rosbags_rviz_plugin
{

class DriveWidget: public QWidget
{
Q_OBJECT
public:
  DriveWidget( std::string text, QWidget* parent = 0);

  virtual void paintEvent( QPaintEvent* event );

  virtual void mousePressEvent( QMouseEvent* event );

  virtual QSize sizeHint() const { return QSize( 150, 150 ); }

  std::string text_;

 Q_SIGNALS:
   void mouseClick( const std::string& );
};

} 


#endif // DRIVE_WIDGET_H
