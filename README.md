# RGBDSlam_Demo
#### A simple RGBD-Slam system, NOT finished yet
练手用，暂时施工中，持续更新XD
参考高博的十四讲结合一些零碎教程伴着Google+StackOverFlow制作，十四讲所有内容都写到一起去了，打算参考ORB-SLAM2的结构，把系统和数据分开。
依赖：OpenCV3、PCL、Eigen3，后续会需要g2o和Ceres，具体见里面的CMakeLists吧..
代码我都加了注释了，有同样跟着十四讲学过的应该很容易看明白，当作参考吧..

#### 完成部分:
0. 按时间戳同步从Ros topic订阅RGB图像和深度图像
1. message到mat转换以及各编码格式转换部分
2. 提取ORB特征点、计算描述子
3. 帧间匹配，PnP求解相机位姿

#### 未完成：
0. 点云拼接,处理，发布
1. 优化：EKF(真的要用这个？)/BA、图优化（还是再看看原理吧...）
2. 后端优化部分做完再添加英文文档吧。
3. 回环检测
4. 添加点有趣的小功能~比如物体检测之类的~
