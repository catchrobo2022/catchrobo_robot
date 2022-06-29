無線でPS4コントローラー(DualShock4)を使う場合は以下のをインストールする必要がある

sudo pip install ds4drv

そのあと起動

sudo ds4drv

ps+shareを押す

rosrun joy joy_node _dev:=/dev/input/js0

これで/joyでtopicを認識できる
※もし、js0かjs1か調べたい場合は下記コマンド
ls /dev/input/js*


参考サイト
ボタン配置
https://cryborg.hatenablog.com/entry/2016/09/19/185501

https://www.sato-susumu.com/entry/tb3-ds4

こいつのボタン配置はガセ
https://qiita.com/Yuya-Shimizu/items/4bed435e65cefc6d2df1




rosrun catchrobo_manual catchrobo_manual_2022.py

roslaunch catchrobo_test test.launch


Commandの
position_min,max
velocity_limit
acceleration_limit
jerk_limit
は適当に設定してます