#ifndef Blue_FIELD_H
#define Blue_FIELD_H

#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <rviz/panel.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Int32MultiArray.h>

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsPathItem>

#include <rviz/message_filter_display.h>
#include <rviz/ogre_helpers/arrow.h>
#include <geometry_msgs/PointStamped.h>

#include <jsk_rviz_plugins/Pictogram.h>

namespace Ui {
class Blue;
}

namespace field
{
class Blue: public rviz::Panel
{
    Q_OBJECT

public:
    //explicit Blue_field(QWidget *parent = 0);
    Blue(QWidget *parent = 0);
    ~Blue() override; 

    int obj[25];
    int gl[18];
    int obj_frame[25] = {0};
    int gl_frame[18] = {0};
    int status=0;
    int touch_mode=0;
    float ti=180.0;
    int stop_ti = 0;
    int type_count = 0;

    std_msgs::Int32MultiArray array_obj;
    std_msgs::Int32MultiArray array_gl;
    std_msgs::Int8 menu;
    std_msgs::Int8 arrow;

    std::string project_path;

    jsk_rviz_plugins::Pictogram pictgram;

    void onInitialize() override;
    void set_icon();
    void onEnable();
    void onDisable();
    void send_obj_msgs(bool flag);
    void send_gl_msgs(bool flag);
    void send_menu_msgs(bool flag);
    void arrayback_obj(const std_msgs::Int32MultiArray& msg);
    void arrayback_gl(const std_msgs::Int32MultiArray& msg);
    void count_score();
    void count_obj();
    void count_gl();
    void timer();
    void marker_obj(int num, int i);
    void marker_gl(int num, int i);
    void jsk_show();

private Q_SLOTS:
  void obj_Clicked();
  void gl_Clicked();
  void menu_panel();
  void reset_menu();
  void touch_rm();
  void touch_tar();
  void countdown();

protected:
  Ui::Blue *ui;
  QGraphicsScene *field_scene;
  QGraphicsScene *goal_scene;
  QGraphicsPixmapItem *field_pix;
  QGraphicsPixmapItem *goal_pix;
  QIcon icon;

  int obj_num = 25;
  int gl_num = 18;
  int sendtime = 100; //ms
  ros::NodeHandle nh_;
  ros::Publisher pub_obj;
  ros::Publisher pub_gl;
  ros::Publisher pub_menu;
  ros::Publisher pub_arrow;
  ros::Publisher pictgram_pub;
  ros::Publisher float_pub;
  ros::Subscriber sub_obj;
  ros::Subscriber sub_gl;
  ros::Timer ti_; 
  ros::NodeHandle n;
};
}
#endif // Blue_FIELD_H
