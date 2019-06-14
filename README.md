 


# 激光雷达障碍物检测和聚类

 ---
   
&nbsp;&nbsp;  
### 参考论文： Efficient Online Segmentation for Sparse 3D Laser Scans ;  
&nbsp;&nbsp;

   
其中包含地面分割代码 my_ransac

&nbsp;&nbsp;
 ---
 
## 依赖安装 install dependence :
numpy
&nbsp;&nbsp;

  
python-pcl（安装参考python-pcl的Github,如果不需要可视化可以不安装，注释掉相应代码即可）
&nbsp;&nbsp;
   

 ---

## 运行 run

直接运行 image_range_cluster_v2.py 即可；
 &nbsp;&nbsp;
python image_range_cluster_v2.py


 ---
 
## 需完善 TODO :

   
1 运行时间较长（~2s），需优化，论文提到只需要20ms,差距还很大；
&nbsp;&nbsp;
    

2 目标聚类效果还有待提高，存在把同一个物体分割成几个部分的情况；
 &nbsp;&nbsp;


    
3 RANSAC 地面分割效果还存在一些问题，针对某些特殊情况会出现漏掉部分地面或者分割出墙面；

 &nbsp;&nbsp;



4 需要对检测出来的物体做一个立体框，同时计算出目标的尺寸；





 ---
   
## 效果图 
      
![avatar](https://github.com/lonlonago/-/blob/master/demo.png)
&nbsp;&nbsp;
