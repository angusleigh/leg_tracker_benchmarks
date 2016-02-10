#include <stdio.h>
#include <math.h>

#include <QPainter>
#include <QMouseEvent>

#include "drive_widget.h"

namespace annotate_rosbags_rviz_plugin
{

DriveWidget::DriveWidget( std::string text, QWidget* parent )
  : QWidget( parent )
  , text_(text)
{
}

void DriveWidget::paintEvent( QPaintEvent* event )
{
  QColor background;
  QColor crosshair;
  background = Qt::white;
  crosshair = Qt::black;

  int w = width();
  int h = height();
  int size = (( w > h ) ? h : w) - 1;
  int hpad = ( w - size ) / 2;
  int vpad = ( h - size ) / 2;

  QPainter painter( this );

  painter.drawRect( QRect( hpad, vpad, size, size ));
  painter.drawText( QRectF(hpad, vpad, size, size), Qt::AlignCenter, tr(text_.c_str()));
}

void DriveWidget::mousePressEvent( QMouseEvent* event )
{
  Q_EMIT mouseClick( text_ );
}
} 
