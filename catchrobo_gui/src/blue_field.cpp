#include "field/blue_field.h"
#include "ui_blue_field.h"

#include <pluginlib/class_list_macros.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <QTimer>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <tf/transform_listener.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include <iostream>


namespace field
{
Blue::Blue(QWidget *parent) :
    rviz::Panel(parent),
    ui(new Ui::Blue)
{
    ui->setupUi(this);
    project_path = ros::package::getPath("catchrobo_gui");
    this->set_icon();
    //this->setStyleSheet("background-color: Blue;");
    ui->shootbox->setStyleSheet("background-color: rgb(0,255,0);");
    ui->comAREA->setStyleSheet("background-color: rgb(255,255,255);");
    ui->Top->setStyleSheet("background-color: rgb(0,0,255);");
    ui->Middle->setStyleSheet("background-color: rgb(0,0,255);");
    ui->Bottom->setStyleSheet("background-color: rgb(0,0,255);");

    for(int num=0;num<obj_num;++num){
      findChild<QFrame*>(QString("fm"+QString::number(num)))->setStyleSheet("color: rgb(255,0,0)");
    }
    for(int num=0;num<gl_num;++num){
    findChild<QFrame*>(QString("frm"+QString::number(num)))->setStyleSheet("color: rgb(255,0,0)");
    }

    ui->bt_add->setChecked(1);

    //rostopic
    pub_obj = nh_.advertise<std_msgs::Int32MultiArray>("obj_rigo", 1);
    sub_obj = n.subscribe("obj_giro", sendtime, &Blue::arrayback_obj, this);

    pub_gl = nh_.advertise<std_msgs::Int32MultiArray>("gl_rigo", 1);
    sub_gl = n.subscribe("gl_giro", sendtime, &Blue::arrayback_gl, this);

    pub_menu = nh_.advertise<std_msgs::Int8>("menu", 1);

    array_obj.data.resize(obj_num);
    array_gl.data.resize(obj_num);

    for(int i=0; i< obj_num;++i){
      this->marker_off_obj(i);
    }
    for(int i=0; i< gl_num;++i){
      this->marker_off_gl(i);
    }
}

Blue::~Blue() = default;

void Blue::onInitialize()
{
  for(int i=0; i<obj_num; i++){
    connect(findChild<QPushButton*>(QString("obj"+QString::number(i))), SIGNAL(clicked()), this, SLOT(obj_Clicked()));
  }
  for(int i=0; i<gl_num; i++){
    connect(findChild<QPushButton*>(QString("gl"+QString::number(i))), SIGNAL(clicked()), this, SLOT(gl_Clicked()));
  }

  connect(findChild<QPushButton*>(QString("bt_init")), SIGNAL(clicked()), this, SLOT(initialize()));
  connect(findChild<QPushButton*>(QString("bt_sta")), SIGNAL(clicked()), this, SLOT(start()));
  connect(findChild<QPushButton*>(QString("bt_pus")), SIGNAL(clicked()), this, SLOT(pause()));
  connect(findChild<QPushButton*>(QString("bt_stp")), SIGNAL(clicked()), this, SLOT(stop()));

  connect(findChild<QPushButton*>(QString("bt_add")), SIGNAL(clicked()), this, SLOT(touch_rm()));
  connect(findChild<QPushButton*>(QString("bt_tar")), SIGNAL(clicked()), this, SLOT(touch_tar()));
  //QTimer::singleShot(sendtime, this, SLOT(hogehoge));
}

void Blue::set_icon()
{
  //add img to QPushButton
  std::string obj_on_img_path = "/img/jagarico_icon.png";
  std::string obj_off_img_path = "/img/jagarico_icon_bw.png";

  std::string obj_on_path_full = project_path+obj_on_img_path;
  std::string obj_off_path_full = project_path+obj_off_img_path;
  QString obj_on_path_Qstr = QString::fromLocal8Bit(obj_on_path_full.c_str());
  QString obj_off_path_Qstr = QString::fromLocal8Bit(obj_off_path_full.c_str());

  icon.addFile(obj_on_path_Qstr, QSize(100,100), QIcon::Normal, QIcon::On);
  icon.addFile(obj_off_path_Qstr, QSize(100,100), QIcon::Normal, QIcon::Off);
  
  for(int i=0;i<obj_num;++i){
    findChild<QPushButton*>(QString("obj"+QString::number(i)))->setIconSize(QSize(45,45));
    findChild<QPushButton*>(QString("obj"+QString::number(i)))->setIcon(icon);
    findChild<QPushButton*>(QString("obj"+QString::number(i)))->setChecked(1);
    findChild<QPushButton*>(QString("obj"+QString::number(i)))->setStyleSheet("background-color: Gray");
  }
  for(int i=0;i<gl_num;++i){
    findChild<QPushButton*>(QString("gl"+QString::number(i)))->setIconSize(QSize(45,45));
    findChild<QPushButton*>(QString("gl"+QString::number(i)))->setIcon(icon);
    findChild<QPushButton*>(QString("gl"+QString::number(i)))->setChecked(0);
    findChild<QPushButton*>(QString("gl"+QString::number(i)))->setStyleSheet("background-color: Gray");
  }
}

void Blue::onEnable()
{
  show();
  parentWidget()->show();
}

void Blue::onDisable()
{
  hide();
  parentWidget()->hide();
}

void Blue::obj_Clicked()
{
  this->count_obj();
  this->send_obj_msgs(false);
}

void Blue::gl_Clicked()
{
  this->count_gl();
  this->send_gl_msgs(false);
  //ROS_INFO("You pushed the button.");
  //ROS_INFO_STREAM(array);
}

void Blue::send_obj_msgs(bool flag)
{
  for(int i=0; i<obj_num; i++){
      array_obj.data[i] = obj[i];
  }
  pub_obj.publish(array_obj);
  //ROS_INFO("You pushed.");
  if(flag == true){
    QTimer::singleShot(sendtime, this, SLOT(send_obj_msgs(true)));
  }
}

void Blue::send_gl_msgs(bool flag)
{
  for(int i=0; i<gl_num; i++){
      array_gl.data[i] = gl[i];
  }
  pub_gl.publish(array_gl);
  if(flag == true){
    QTimer::singleShot(sendtime, this, SLOT(send_gl_msgs(true)));
  }
}

void Blue::send_menu_msgs(bool flag)
{
  menu.data = status;
  pub_menu.publish(menu);
  if(flag == true){
    QTimer::singleShot(sendtime, this, SLOT(send_gl_msgs(true)));
  }
}

void Blue::arrayback_obj(const std_msgs::Int32MultiArray& msg){
  int num = msg.data.size();
  for(int i=0; i<num; i++){
      findChild<QPushButton*>(QString("obj"+QString::number(i)))->setChecked(msg.data[i]);
      this->marker_off_obj(i);
      if(msg.data[i] == 2){
        this->marker_on_obj(i);
      }
  }
  this->count_obj();
}

void Blue::arrayback_gl(const std_msgs::Int32MultiArray& msg){
  int num = msg.data.size();
  //for (int i = 0; i < num; i++)
  //{
  //  ROS_INFO("[%i]:%d", i, msg.data[i]);
  //}
  for(int i=0; i<num; i++){
      findChild<QPushButton*>(QString("gl"+QString::number(i)))->setChecked(msg.data[i]);
      this->marker_off_gl(i);
      if(msg.data[i] == 2){
        this->marker_on_gl(i);
      }
  }
  this->count_gl();
}

void Blue::count_obj(){
  for(int i=0; i<obj_num; i++){
      obj[i] = findChild<QPushButton*>(QString("obj"+QString::number(i)))->isChecked();
  }
}

void Blue::count_gl(){
  for(int i=0; i<gl_num; i++){
      gl[i] = findChild<QPushButton*>(QString("gl"+QString::number(i)))->isChecked();
  }
  this->count_score();
}

void Blue::initialize(){
  status = 0;
  ui->bt_init->setChecked(1);
  ui->bt_sta->setChecked(0);
  ui->bt_pus->setChecked(0);
  ui->bt_stp->setChecked(0);
  this->send_menu_msgs(false);
  ti = 180.0;
  this->timer();
}
void Blue::start(){
  status = 1;
  ui->bt_init->setChecked(0);
  ui->bt_sta->setChecked(1);
  ui->bt_pus->setChecked(0);
  ui->bt_stp->setChecked(0);
  this->send_menu_msgs(false);
  stop_ti = 0;
  QTimer::singleShot(100, this, SLOT(countdown()));
}
void Blue::pause(){
  status = 2;
  ui->bt_init->setChecked(0);
  ui->bt_sta->setChecked(0);
  ui->bt_pus->setChecked(1);
  ui->bt_stp->setChecked(0);
  this->send_menu_msgs(false);
}
void Blue::stop(){
  status = 3;
  ui->bt_init->setChecked(0);
  ui->bt_sta->setChecked(0);
  ui->bt_pus->setChecked(0);
  ui->bt_stp->setChecked(1);
  this->send_menu_msgs(false);
  stop_ti = 1;
  this->timer();
}
void Blue::touch_rm(){
  touch_mode = 0;
  ui->bt_add->setChecked(1);
  ui->bt_tar->setChecked(0);
}
void Blue::touch_tar(){
  touch_mode = 1;
  ui->bt_add->setChecked(0);
  ui->bt_tar->setChecked(1);
}
void Blue::count_score(){
  int score = 0;
  int bonus = 1;
  for(int i=0; i<gl_num; i++){
      score += gl[i];
      bonus *= gl[i];
      if((i+1)%6 == 0){
        if(bonus == 1){
          score += 3;
        }
        bonus = 1;
      }
  }
  ui->Score->display(score);
}

void Blue::timer(){
  char buffer[7];
  sprintf(buffer, "%.1f", ti);
  ui->Time->display(buffer);
}

void Blue::countdown(){
  ti -= 0.1;
  if(ti >= 0 && stop_ti == 0){
    this->timer();
    QTimer::singleShot(100, this, SLOT(countdown()));
  }
}

void Blue::marker_off_obj(int num){
  if(num >= 0 && num < obj_num){
    findChild<QFrame*>(QString("fm"+QString::number(num)))->setLineWidth(0);
  }
}
void Blue::marker_on_obj(int num){
  if(num >= 0 && num < obj_num){
    findChild<QFrame*>(QString("fm"+QString::number(num)))->setLineWidth(1);
  }
}
void Blue::marker_off_gl(int num){
  if(num >= 0 && num < obj_num){
    findChild<QFrame*>(QString("frm"+QString::number(num)))->setLineWidth(0);
  }
}
void Blue::marker_on_gl(int num){
  if(num >= 0 && num < obj_num){
    findChild<QFrame*>(QString("frm"+QString::number(num)))->setLineWidth(1);
  }
}

}

PLUGINLIB_EXPORT_CLASS(field::Blue, rviz::Panel )
