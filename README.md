
# 激光雷达障碍物检测和聚类

&nbsp;&nbsp;
参考论文： Efficient Online Segmentation for Sparse 3D Laser Scans ;  
&nbsp;&nbsp;


其中包含地面分割代码 my_ransac
&nbsp;&nbsp;

依赖安装 install dependence :
numpy
&nbsp;&nbsp;


python-pcl（安装参考python-pcl的Github）
&nbsp;&nbsp;

#需完善 TODO :
1 运行时间较长，需优化；
&nbsp;&nbsp;


2 目标聚类效果还有待提高，会把同一个物体分割成几个部分；
&nbsp;&nbsp;

3 RANSAC 地面分割效果还存在一些问题，针对某些特殊情况会出现漏掉部分地面或者分割出墙面；



![avatar](https://github.com/lonlonago/-/blob/master/demo.png)
&nbsp;&nbsp;
