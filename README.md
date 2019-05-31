# ricoh_theta_ros

## THETAの設定
[YouTube](https://www.youtube.com/watch?v=O7-LXnPuFU0#action=share)を参考にしてくだい。

## ROSの設定
```
$ sudo apt install libuvc_camera

$ cd ~/catkin_ws/src/

$ git clone https://github.com/KoutaOhishi/ricoh_theta_ros.git

$ chmod 755 ricoh_theta_ros/src/*

$ cd ~/catkin_ws/

$ catkin_make
```

## 起動方法
```
$ roslaunch ricoh_theta_ros camera.launch
```

## THETAから得られる画像について
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
