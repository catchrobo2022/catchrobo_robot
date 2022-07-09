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



手動プログラム
# rosrun catchrobo_manual catchrobo_manual_2022.py
# rosrun catchrobo_manual catchrobo_manual_2022_main.py
# roslaunch catchrobo_manual manual.launch field:="red"

roslaunch catchrobo_manual manual_xbox.launch

roslaunch catchrobo_manual manual_ps4.launch


メインプログラム
# roslaunch catchrobo_test test.launch
# rosrun catchrobo_test mbed_sim_test.py

roslaunch catchrobo_bringup sim_bringup.launch field:="red" 




開発記録

0612
Commandの
jerk_limit
は適当に設定してます
position_min,max
velocity_limit
acceleration_limit
はexcel通りにやった

0613
PS4コントローラのクラスをパッケージ化
excelの位置制限とかのパラメータの入力（m単位）

0615
touch pad でボタン配置を表示機能実装

0616
pos, vel acc, jerkのすべての指令値を/RADIUSで割って、長さから角度に変換
これいらなくなった

0616-2
feature msgと統合
新しい指令仕様に変更
gripperの値は適当
joint_state topicのsubscribeを記載
位置制御を現在位置から動かす感じに修正（未検証）
長い名前のself変数を短い代用の変数に置き換えて見やすくした（未検証）
四角ボタンで原点に戻る機能の追加（未検証）

0617
移動が遅い理由を調査　kp kdの値調整
位置制御を現在位置から動かす感じに修正（未検証）全然できない
長い名前のself変数を短い代用の変数に置き換えて見やすくした（検証）okay 処理にあんまり影響なさそう

0618
位置制御を現在位置から動かす感じに修正できた
四角ボタンで原点に戻る機能の追加（検証）okay 部分的に成功　一応失敗

0619
gripperのところを丸だけで実装（グリッパーが動かないから検証はできてない）topicは正しく送れてる
pause機能の実装
自動手動切り替えの実装中

0625
自動切り替え用のtopicを作った
フィールドを対応をした

0702
仕様変更で指令のtopicをros_cmdで送るようになった。
