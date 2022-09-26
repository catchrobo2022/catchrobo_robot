# RaspberryPi
## 環境構築
1. SDカード作成
    - Raspberry Pi Imagerを使う
    - Ubuntu20.04 Server 32Bit版を選択
    - 詳細設定にてSSH、ネットワーク設定をしておく
2. 初回起動
    - SDカードを挿入して起動
    - cloud-initの設定が終わるまで待つ
3. SSH接続
    ```
    ssh root@192.168.xxx.xxx
    ```
    - Windowsの場合、Tera TermからSSH接続する
        - ホスト名: root
    - rootユーザーに切り替え
        ```
        sudo su
        ```
4. ROSインストール
http://wiki.ros.org/action/info/noetic/Installation/Ubuntu
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update
sudo apt install ros-noetic-desktop

source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update

sudo apt install python3-catkin-tools

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin build
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

5. 依存パッケージの追加
```
sudo apt install python3-pandas
sudo apt install ros-noetic-rosserial
sudo apt install ros-noetic-rosserial-mbed
sudo apt install ros-kinetic-geometry2
sudo apt install ros-noetic-jsk-visualization
sudo apt install ros-noetic-joy
sudo apt install python3-rostopic
sudo apt install python3-rosservice
```

6. catchroboレポジトリの追加
```
cd ~/catkin_ws/src
git clone --depth 1 https://github.com/catchrobo2022/catchrobo_robot.git
cd ~/catkin_ws/src/catchrobo_robot
git checkout develop
catkin build
source ~/catkin_ws/devel/setup.bash
```
全パッケージがビルド成功したことを確認する。

7. IPの固定
    - /etc/netplanのファイルに設定を書く
    ```
    network:
    version: 2
    wifis:
        renderer: networkd
        wlan0:
            access-points:
                "SSID名":
                    password: "パスワード"
            dhcp4: false
            optional: true
            addresses: [192.168.xxx.xxx/24]
            gateway4: 192.168.xxx.zzz
            nameservers:
                 addresses: [192.168.xxx.zzz]
    ```
    ```
    sudo netplan apply
    ```
8. GUI導入
```
sudo apt update
sudo apt upgrade
sudo apt install ubuntu-desktop
```
9. ROS_MASTER_URIの設定
```
export ROS_MASTER_URI=http://192.168.xxx.xxx:11311
```

### ラズパイエラー処理
- デザリングスマホ側のネットワークが変わったら新IPを/etc/netplanに設定
- [hostsの設定方法参考サイト](https://i-think-it.net/linux-hosts-name-resolution/)
- [fsckエラー対処参考サイト](https://forums.ubuntulinux.jp/viewtopic.php?id=18660)
