#ifndef TELEOP_PANEL_H
#define TELEOP_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

class QLineEdit;

namespace annotate_rosbags_rviz_plugin
{

class DriveWidget;

class TeleopPanel: public rviz::Panel
{

Q_OBJECT
public:
  TeleopPanel( QWidget* parent = 0 );

public Q_SLOTS:
  void pubMouseClick( const std::string& text );

  void pubIDNum();


protected:
  QLineEdit* output_topic_editor_;

  DriveWidget* drive_widget_1_;
  DriveWidget* drive_widget_2_;
  DriveWidget* drive_widget_3_;
  DriveWidget* drive_widget_4_;
  DriveWidget* drive_widget_5_;
  DriveWidget* drive_widget_6_;
  DriveWidget* drive_widget_7_;

  ros::Publisher annotation_control_pub_;
  ros::Publisher annotation_id_pub_;

  ros::NodeHandle nh_;
};

} 

#endif // TELEOP_PANEL_H
