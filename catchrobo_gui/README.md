# rviz_plugin_examples
Examples for rviz plugin for `melodic` environment.

# Build
```bash
$ mkdir <catkin_ws>/src
$ cd <catkin_ws>/src
$ git clone -b melodic-devel https://github.com/RyodoTanaka/rviz_plugin_examples.git
$ cd <catkin_ws>
$ rosdep install -i -y -r --from-paths src
$ catkin build
$ source devel/setup.bash
```
環境構築が終わってるなら
```bash
$ catkin build
```
だけで大丈夫です

# How to Use
buildした後にrvizのPanels->Add New Panel->Blue or Redで使用可能です
画面の解像度によってはrvizが最大サイズで表示できなくなるため、そういう人がいたら言ってください

# Rostopic
数字の定義は以下のとおりです
- オブジェクトなし : 0
- オブジェクトあり : 1
- 次のターゲット : 2

rigo->ros input gui output

giro->gui input ros output
## obj_rigo（27個のInt32MultiArray）
フィールド上のオブジェクト情報を渡します
## obj_giro（27個のInt32MultiArray）
フィールド上のオブジェクト情報を受け取ります
## gl_rigo（18個のInt32MultiArray）
シュートボックス上のオブジェクト情報を渡します
## gl_giro（18個のInt32MultiArray）
シュートボックス上のオブジェクト情報を受け取ります
## menu（Int8）
メニューボタンの情報を渡します
- init : 0
- start : 1
- pause : 2
- stop : 3
