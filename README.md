# test
这是用于测试雷达相机的中间节点

## camera_node
读取**MP4**格式的视频，并发布到`/image_for_radar`话题上
运行：
```
ros2 run camera camera_node
```

## transform_compressedImg2Img
### 功能
将压缩后的图像转换为普通图像，发布到`/image_for_radar`话题上
主要对东大的rosbag处理，以测试match
### 运行
```
ros2 run camera transform_node
```
### 性能
处理时间在60ms左右

## webm_camera_node
读取**webm**格式的视频，并发布到`/image_for_radar`话题上
主要对25总决赛的录屏处理，以测试网络是否对轮腿有效
运行：
```
ros2 run camera webm_camera_node
```

## sub
获取`/image_for_radar`、`/detector/img_detect`、`/detector/img_car`、`/detector/img_armor`、`/match_draw`话题上的图像，并imshow
运行：
```
ros2 run camera sub_node
```