# ricoh_theta_ros
360度撮影できるカメラ（THETA）をROSで使えるようにするpackage。  
USB接続とwifi接続の2種類のパターンがある。  

## THETAの設定&起動方法
- USB接続  
  [YouTube](https://www.youtube.com/watch?v=O7-LXnPuFU0#action=share)

- WiFi接続  
  [公式サイト](https://support.theta360.com/ja/manual/v/content/prepare/prepare_06.html)

## ROSの設定
```
$ sudo apt install libuvc_camera

$ cd ~/catkin_ws/src/

$ git clone https://github.com/KoutaOhishi/ricoh_theta_ros.git

$ chmod 755 ricoh_theta_ros/src/*

$ cd ~/catkin_ws/

$ catkin_make
```

## Network設定(wifi stream用)
① WifiネットワークでTHETAに接続する。
  - SSIDの例 ： THETAXS1234567.OSC
  - Password : SSIDの数字がpassになる(上のだと1234567がpass)

② 接続名を"THETA"に変更する。  


## 起動方法
- USB接続
  ```
  $ roslaunch ricoh_theta_ros camera.launch
  ```

- WiFi接続
  ```
  $ roscore

  $ rosrun ricoh_theta_ros wifi_switcher.py

  $ rosrun ricoh_theta_ros wifi_switcher.py
  ```

## THETAから得られる画像について(USB接続用)
THETAから送られてくる sensor_msgs/Image型の画像は、Dual-Fisheyeという種類の画像になっています。  
このままだと画像処理には向かないので、変換するノードを作りました。  

- Dual-Fisheye -> Equirectangular(正距円筒図法) written in python
  ```
  $ roslaunch ricoh_theta_ros service.launch
  ```
  画像の変換に１秒くらいかかりますが、綺麗に展開できます。  


- Dual-Fisheye -> Equirectangular(正距円筒図法) written in c++
  ```
  rosrun ricoh_theta_ros stitching
  ```
  修正中  


- Dual-Fisheye -> Perspective(透視投影)
  ```
  rosrun ricoh_theta_ros perspective_converter
  ```
  topicのsubscribeの遅延がありますが、処理は速いです。  
  歪みが多いので物体検出には向かないかも・・・。  

## 参考
- http://blog.livedoor.jp/tmako123-programming/archives/50769806.html  
- https://qiita.com/dcm_kitade/items/6ff790fbb0a97913265b  
