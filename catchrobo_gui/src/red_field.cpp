#include "field/red_field.h"
#include "ui_red_field.h"

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
Red::Red(QWidget *parent) :
    rviz::Panel(parent),
    ui(new Ui::Red)
{
    ui->setupUi(this);
    project_path = ros::package::getPath("catchrobo_gui");
    this->set_icon();
    this->reset_menu();
    //this->setStyleSheet("background-color: Red;");
    ui->shootbox->setStyleSheet("background-color: rgb(0,255,0);");
    ui->comAREA->setStyleSheet("background-color: rgb(210,210,210);");
    ui->Top->setStyleSheet("background-color: rgb(255,0,0);");
    ui->Middle->setStyleSheet("background-color: rgb(255,0,0);");
    ui->Bottom->setStyleSheet("background-color: rgb(255,0,0);");

    for(int num=0;num<obj_num;++num){
      if(num<9){
        findChild<QFrame*>(QString("fm"+QString::number(num)))->setStyleSheet("color: rgb(210,210,210)");
      }else{
        findChild<QFrame*>(QString("fm"+QString::number(num)))->setStyleSheet("color: rgb(255,0,0)");
      }
      findChild<QFrame*>(QString("fm"+QString::number(num)))->setLineWidth(2);
    }
    for(int num=0;num<gl_num;++num){
      findChild<QFrame*>(QString("frm"+QString::number(num)))->setStyleSheet("color: rgb(0,255,0)");
      findChild<QFrame*>(QString("frm"+QString::number(num)))->setLineWidth(2);
    }

    //rostopic
    pub_obj = nh_.advertise<std_msgs::Int32MultiArray>("obj_rigo", 1);
    sub_obj = n.subscribe("obj_giro", sendtime, &Red::arrayback_obj, this);

    pub_gl = nh_.advertise<std_msgs::Int32MultiArray>("gl_rigo", 1);
    sub_gl = n.subscribe("gl_giro", sendtime, &Red::arrayback_gl, this);

    pub_menu = nh_.advertise<std_msgs::Int8>("menu", 1);
    pub_arrow = nh_.advertise<std_msgs::Int8>("arrow_sub", 1);

    arrow.data = 0;
    pub_arrow.publish(arrow);

    array_obj.data.resize(obj_num);
    array_gl.data.resize(obj_gl);
    /*
    for(int i=0; i< obj_num;++i){
      this->marker_obj(i, 0);
    }
    for(int i=0; i< gl_num;++i){
      this->marker_gl(i, 0);
    }
    */
   mytimer.setInterval(1000);
}

Red::~Red() = default;

void Red::onInitialize()
{
  for(int i=0; i<obj_num; i++){
    connect(findChild<QPushButton*>(QString("obj"+QString::number(i))), SIGNAL(clicked()), this, SLOT(obj_Clicked()));
  }
  for(int i=0; i<gl_num; i++){
    connect(findChild<QPushButton*>(QString("gl"+QString::number(i))), SIGNAL(clicked()), this, SLOT(gl_Clicked()));
  }

  connect(findChild<QPushButton*>(QString("origin")), SIGNAL(clicked()), this, SLOT(menu_panel()));
  connect(findChild<QPushButton*>(QString("calib")), SIGNAL(clicked()), this, SLOT(menu_panel()));
  connect(findChild<QPushButton*>(QString("init")), SIGNAL(clicked()), this, SLOT(menu_panel()));
  connect(findChild<QPushButton*>(QString("start")), SIGNAL(clicked()), this, SLOT(menu_panel()));
  
  connect(&mytimer, SIGNAL(timeout()), this, SLOT(countdown()));
  //mytimer.start();
}

void Red::set_icon()
{
  //add img to QPushButton
  std::string obj_on_img_path = "/img/jagarico_icon2.png";
  std::string obj_off_img_path = "/img/jagarico_icon2_bw.png";

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

void Red::onEnable()
{
  show();
  parentWidget()->show();
}

void Red::onDisable()
{
  hide();
  parentWidget()->hide();
}

void Red::obj_Clicked()
{
  if(touch_mode == 1){
    int arg;
    QPushButton * btn = dynamic_cast<QPushButton*>(sender());
    for( int i=0; i<obj_num; ++i){
      if(QString("obj"+QString::number(i)) == btn->objectName()){
        arg = i;
      }
    }
    for(int i=0; i< obj_num;++i){
      this->marker_obj(i, 0);
    }
    btn->setChecked(1);
    this->marker_obj(arg, 1);
  }
  
  this->count_obj();
  this->send_obj_msgs(false);
}

void Red::gl_Clicked()
{
  if(touch_mode == 1){
    int arg;
    QPushButton * btn = dynamic_cast<QPushButton*>(sender());
    for( int i=0; i<obj_num; ++i){
      if(QString("gl"+QString::number(i)) == btn->objectName()){
        arg = i;
      }
    }  
    for(int i=0; i< gl_num;++i){
      this->marker_gl(i, 0);
    }
    btn->setChecked(0);
    this->marker_gl(arg, 1);
  }
  this->count_gl();
  this->send_gl_msgs(false);
  //ROS_INFO("You pushed the button.");
  //ROS_INFO_STREAM(array);
}

void Red::send_obj_msgs(bool flag)
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

void Red::send_gl_msgs(bool flag)
{
  for(int i=0; i<gl_num; i++){
      array_gl.data[i] = gl[i];
  }
  pub_gl.publish(array_gl);
  if(flag == true){
    QTimer::singleShot(sendtime, this, SLOT(send_gl_msgs(true)));
  }
}

void Red::send_menu_msgs(bool flag)
{
  menu.data = status;
  pub_menu.publish(menu);
  if(flag == true){
    QTimer::singleShot(sendtime, this, SLOT(send_gl_msgs(true)));
  }
}

void Red::arrayback_obj(const std_msgs::Int32MultiArray& msg){
  int num = msg.data.size();
  if(num != obj_num){
    ROS_INFO("The number of elements is different");
    ROS_INFO("need: %d ,but %d recieved", obj_num,num);
  }else{
    for(int i=0; i<num; i++){
        findChild<QPushButton*>(QString("obj"+QString::number(i)))->setChecked(msg.data[i]);
        this->marker_obj(i, 0);
        if(msg.data[i] == 2){
          this->marker_obj(i, 1);
        }
    }
    this->count_obj();
  }
}

void Red::arrayback_gl(const std_msgs::Int32MultiArray& msg){
  int num = msg.data.size();
  if(num != gl_num){
    ROS_INFO("The number of elements is different");
    ROS_INFO("need: %d ,but %d recieved", gl_num,num);
  }else{
    for(int i=0; i<num; i++){
        findChild<QPushButton*>(QString("gl"+QString::number(i)))->setChecked(msg.data[i]);
        this->marker_gl(i, 0);
        if(msg.data[i] == 2){
          this->marker_gl(i, 1);
        }
    }
    this->count_gl();
  }
}

void Red::count_obj(){
  for(int i=0; i<obj_num; i++){
      obj[i] = findChild<QPushButton*>(QString("obj"+QString::number(i)))->isChecked() + obj_frame[i];
  }
}

void Red::count_gl(){
  for(int i=0; i<gl_num; i++){
      gl[i] = findChild<QPushButton*>(QString("gl"+QString::number(i)))->isChecked() + gl_frame[i];
  }
  this->count_score();
}

void Red::menu_panel(){
  QPushButton * btn = dynamic_cast<QPushButton*>(sender());
  QString name = btn->objectName();
  if(QString("origin") == name){
    if(QString("origin") == btn->text()){
      btn->setText("Really?");
      status = 0;
    }else{
      status = 1;
      btn->setText("origin");
      //ui->origin->setEnabled(0);
      //ui->calib->setEnabled(1);
      //ui->init->setEnabled(1);
    }
    ui->origin->setChecked(0);
  }else if(QString("calib") == name){
    if(QString("1st point") == btn->text()){
      status = 3;
      arrow.data = 2;
      btn->setText("2nd point");
    }else if(QString("2nd point") == btn->text()){
      status = 4;
      arrow.data = 3;
      btn->setText("3rd point");
    }else if(QString("3rd point") == btn->text()){
      status = 5;
      arrow.data = 1;
      btn->setText("1st point");
    }else{
      arrow.data = 1;
      btn->setText("1st point");
      status = 2;
    }
    ui->calib->setChecked(0);
    pub_arrow.publish(arrow);
  }else if(QString("init") == name){
    ui->calib->setText("2.Calib");
    status = 6;
    arrow.data = 0;
    pub_arrow.publish(arrow);
    ui->init->setChecked(0);
    //ui->calib->setEnabled(0);
    //ui->init->setEnabled(0);
    //ui->start->setEnabled(1);
  }else if(QString("start") == name){
    status = 7;
    mytimer.stop();
    ti = 180.0;
    stop_ti = 0;
    ui->Time->setStyleSheet("color: rgb(0,0,0)");
    mytimer.start();
    ui->start->setChecked(0);
    //ui->start->setEnabled(0);
    //ui->origin->setEnabled(1);
  }

  this->send_menu_msgs(false);
  this->timer();
}

void Red::reset_menu(){
  ui->origin->setEnabled(1);
  ui->calib->setEnabled(1);
  ui->init->setEnabled(1);
  ui->start->setEnabled(1);
}

void Red::count_score(){
  int score = 0;
  int bonus = 1;
  for(int i=0; i<gl_num; i++){
    if(gl[i]!=2){
      score += gl[i];
      bonus *= gl[i];
    }else{
      bonus = 0;
    }
    if((i+1)%6 == 0){
      if(bonus == 1){
        score += 3;
      }
      bonus = 1;
    }
  }
  if(score >= 27){
    stop_ti = 1;
  }else{
    stop_ti = 0;
  }
  ui->Score->display(score);
}

void Red::timer(){
  if(ti<0){
    ui->Time->setStyleSheet("color: rgb(255,0,0)");
  }
  char buffer[7];
  sprintf(buffer, "%.0f", ti);
  ui->Time->display(buffer);
}

void Red::countdown(){
  ti -= 1;
  if(stop_ti == 0){
    this->timer();
  }
}

void Red::marker_obj(int num, int i){
  for(int i=0;i<obj_num;++i){
    if(i<9){
      findChild<QFrame*>(QString("fm"+QString::number(i)))->setStyleSheet("color: rgb(210,210,210)");
    }else{
      findChild<QFrame*>(QString("fm"+QString::number(i)))->setStyleSheet("color: rgb(255,0,0)");
    }
  }
  findChild<QFrame*>(QString("fm"+QString::number(num)))->setStyleSheet("color: rgb(255,170,0)");
  obj_frame[num] = i*2;
}
void Red::marker_gl(int num, int i){
  for(int i=0;i<gl_num;++i){
    findChild<QFrame*>(QString("frm"+QString::number(i)))->setStyleSheet("color: rgb(0,255,0)");
  }
  findChild<QFrame*>(QString("frm"+QString::number(num)))->setStyleSheet("color: rgb(255,170,0)");
  gl_frame[num] = i*2;
}

}

PLUGINLIB_EXPORT_CLASS(field::Red, rviz::Panel )
