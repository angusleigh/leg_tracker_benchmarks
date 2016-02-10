#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>


#include "drive_widget.h"
#include "teleop_panel.h"

namespace annotate_rosbags_rviz_plugin
{

TeleopPanel::TeleopPanel( QWidget* parent )
  : rviz::Panel( parent )
{

  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "ID num:" ));
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_ );

  drive_widget_1_ = new DriveWidget("Clear\nprevious");  
  drive_widget_2_ = new DriveWidget("Next");
  drive_widget_3_ = new DriveWidget("Forwards");
  drive_widget_4_ = new DriveWidget("End");
  drive_widget_5_ = new DriveWidget("Next\nmiss");
  drive_widget_6_ = new DriveWidget("Next\nid_switch");
  drive_widget_7_ = new DriveWidget("Next\nfp");


  QHBoxLayout* button_layout = new QHBoxLayout;
  button_layout->addWidget( drive_widget_1_ );
  button_layout->addWidget( drive_widget_2_ );
  button_layout->addWidget( drive_widget_3_ );
  button_layout->addWidget( drive_widget_4_ );

  QHBoxLayout* button_layout_2 = new QHBoxLayout;
  button_layout_2->addWidget( drive_widget_5_ );
  button_layout_2->addWidget( drive_widget_6_ );
  button_layout_2->addWidget( drive_widget_7_ );

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(topic_layout);
  layout->addLayout(button_layout);
  layout->addLayout(button_layout_2);


  setLayout( layout );

  connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( pubIDNum() ));
  connect( drive_widget_1_, SIGNAL(mouseClick( const std::string& )), this, SLOT(pubMouseClick( const std::string& )));
  connect( drive_widget_2_, SIGNAL(mouseClick( const std::string& )), this, SLOT(pubMouseClick( const std::string& )));
  connect( drive_widget_3_, SIGNAL(mouseClick( const std::string& )), this, SLOT(pubMouseClick( const std::string& )));
  connect( drive_widget_4_, SIGNAL(mouseClick( const std::string& )), this, SLOT(pubMouseClick( const std::string& )));
  connect( drive_widget_5_, SIGNAL(mouseClick( const std::string& )), this, SLOT(pubMouseClick( const std::string& )));
  connect( drive_widget_6_, SIGNAL(mouseClick( const std::string& )), this, SLOT(pubMouseClick( const std::string& )));
  connect( drive_widget_7_, SIGNAL(mouseClick( const std::string& )), this, SLOT(pubMouseClick( const std::string& )));

  annotation_control_pub_ = nh_.advertise<std_msgs::String>("annotation_control", 1);
  annotation_id_pub_ = nh_.advertise<std_msgs::Int32>("annotation_id", 1);
}

void TeleopPanel::pubMouseClick( const std::string& text )
{
  if(ros::ok())
  {
    std_msgs::String msg;
    msg.data = text;
    annotation_control_pub_.publish( msg );
  }
}

void TeleopPanel::pubIDNum()
{
  if(ros::ok())
  {
    std_msgs::Int32 msg;
    msg.data = atoi(output_topic_editor_->text().toStdString().c_str()); // this is ugly.
    annotation_id_pub_.publish( msg );
  }
}
} 

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(annotate_rosbags_rviz_plugin::TeleopPanel,rviz::Panel )
