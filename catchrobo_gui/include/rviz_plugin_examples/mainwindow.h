#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <rviz/panel.h>

#include <QMainWindow>
#include <QWidget>

namespace Ui {
class Blue;
}

namespace rviz_plugin_examples
{
class ObjPanel: public rviz::Panel
{
  Q_OBJECT
 public:
  ObjPanel(QWidget* parent = 0);
  ~ObjPanel() override;

  void onInitialize() override;
  void onEnable();
  void onDisable();

private Q_SLOTS:
  void buttonClicked();

protected:
  Ui::Blue* ui_;
  int value_{0};
  std::string topic_name_{"object"};

  ros::NodeHandle nh_;
  ros::Publisher pub_;
};
} // end namespace rviz_plugin_examples