# Catchrobo 2021 ROS Package

## Environment
- ubuntu 18
- ros melodic

## Requirement
- sudo apt install ros-melodic-rosserial-mbed


## How to use
### Launching simulated robot
1. Launch moveit
```
roslaunch catchrobo_description catchrobo_display.launch gui:=True field:=red

```

### mbed simulator demo
```
roslaunch catchrobo_test test.launch
rosrun catchrobo_test mbed_sim_test.py # commandをpublishするだけのテストドライバ。自由に書き換え可能
```

シミュレーターの位置更新は下の式で実行される。kp, kdの値を変えるとそれっぽい変化をする.
```
p += cmd.kp * (cmd.p - p) + cmd.kd * cmd.v * dt
```


## mbedへの移行
1. catchrobo_sim/include/catchrobo_simをzip化する
1. mbed compilerでプログラム名を選択し右クリック
1. インポートを選択
1. アップロードタブを選択
1. 下にあるChoose Fileから作成したzipを選択
1. ライブラリとしてインポートする(古いものがある場合には、削除してからインポートする)

- msgが変化した場合
``` 
rosrun rosserial_mbed make_libraries.py <コードの生成場所。どこでもいい>
```
作成後、round関数がmbedにないらしいので ros_lib/ros/duration.h　およびros_lib/ros/time.h　のclassの中に
```
 double round(double number) { return number < 0.0 ? ceil(number - 0.5): floor(number + 0.5); };
```
を追加する。

作成したros_libを上記の方法でインポートする。


https://os.mbed.com/teams/catchrobo2022/


```
roslaunch catchrobo_description catchrobo_display.launch 

sudo chmod a+rw /dev/ttyACM0 
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
rosrun catchrobo_test mbed_sim_test.py
```

Xbeeあり
```
roslaunch catchrobo_description catchrobo_display.launch 

sudo chmod a+rw /dev/ttyUSB0 
rosrun catchrobo_driver serial_node_float.py _port:=/dev/ttyUSB0 _baud:=115200
rosrun catchrobo_test mbed_sim_test.py
```

printfしたいなら
```
cu -s 921600 -l /dev/ttyACM0
```


- [x] jointをradに
- [x] finishを変える
- [x] simulatorをそれっぽく : 失敗。摩擦の項とかが必要で大変
- [ ] meter rad変換つくる


wifiあるところで
- push
- [ ] 井上に 大体 0.002 * 54/(2*math.pi)倍されてしまうこと(rad管理になったこと)を伝える
- [ ] mbedで原点設定する方法見る

帰国後
- [ ] enable / disable 検証
- [ ] 原点だし
    - [ ] パラメータ探し
    - [ ] 設定されるか検証
    