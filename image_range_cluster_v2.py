# coding:utf-8

from data_gen import lidar_to_2d_front_view_depth, _normalize
from my_ransac import my_ransac_v2
import numpy as np
import time


import pcl.pcl_visualization


def depth_clustering(projected_image,
                     angle_tollerance = np.radians(8),
                     min_cluster_size = 100,
                     max_cluster_size = 25000,
                     h_res=np.radians(360/2048.),
                     v_res=np.radians(26.9/65)):

    depth = projected_image[:, :, 4]
    depth_shape = depth.shape

    # 初始 label , label matrix
    label = 1
    label_matrix = np.zeros(depth_shape)

    # 对每个点进行检测，同时聚类
    for i in range(depth_shape[0]):
        for j in range(depth_shape[1]):
            if label_matrix[i, j] == 0:
                if depth[i, j] == 0:
                    continue  # 地面点跳过计算
                one_label_BFS(i, j, label, label_matrix,
                              depth, angle_tollerance,
                              h_res, v_res, projected_image)
                label += 1


    # 过滤掉点数量较多或较少的类别
    not_real_obstacle = 0
    for lx in range(1,label):
        cluster_num = np.sum(label_matrix == lx)
        # 最大数量限制会把一些墙体也过滤掉，这是否是需要的？
        # if cluster_num < min_cluster_size :
        if cluster_num < min_cluster_size or cluster_num > max_cluster_size:
            # 把过滤掉的类别全部设置为地面
            label_matrix[label_matrix == lx] = 0
            not_real_obstacle += 1
    # print('After filtered  left ', label - not_real_obstacle, ' real clusters .')

    return label_matrix


def one_label_BFS(i, j, label, label_matrix,
                  depth, angle_tollerance,
                  hres, vres, projected):
    # 不管是广度优先还是深度优先，都是要搜索到所有与它关联的节点；
    # 它是怎么保证最多搜索次数只有2N？
    # TODO 用set 更合适，因为会添加进去很多重复的点，可以用set的自动去重；
    Q = [(i,j)]
    while len(Q) > 0:
        r,c = Q[0]
        Q.pop(0) # 默认是丢弃最后一个元素， 要放在这里，不然会死循环
        if label_matrix[r, c] > 0:
            # we have already labeled this point.No need to deal it.
            continue
        # label 赋值
        label_matrix[r, c] = label

        # 深度值为0，不纳入计算
        if depth[r,c] < 0.0001:
            continue

        for rn, cn in neighbourhood(r, c):
            if rn >= 65 or rn < 0 or cn < 0 or cn >= 2049:
                continue  # 超出范围的点，不考虑
            if label_matrix[rn, cn] > 0:
                # we have already labeled this point.No need to add it.
                continue
            d1 = np.max((depth[r,c], depth[rn, cn]))
            d2 = np.min((depth[r,c], depth[rn, cn]))

            # TODO  两种方案，使用固定分辨率，或者实时计算分辨率，好像差别不大
            # plan 1
            if r != rn:
                res = vres
            elif c != cn:
                res = hres

            # plan 2
            # if r != rn:
            #     # y_img_2 = -np.arcsin(z_lidar/d) # 得到垂直方向角度
            #     res0 = np.arcsin(projected[r, c, 2] / np.sqrt(projected[r, c, 0] ** 2 + projected[r, c, 1] ** 2 + projected[r, c, 2] ** 2))
            #     res1 = np.arcsin(projected[rn, cn, 2] / np.sqrt(projected[rn, cn, 0] ** 2 + projected[rn, cn, 1] ** 2 + projected[rn, cn, 2] ** 2))
            #     res00 = np.arcsin(projected[r, c, 2] / depth[r,c])
            #     res11 = np.arcsin(projected[rn, cn, 2] / depth[rn, cn])
            #     print(res0,res00,res1,res11)
            #     res = abs(res0 - res1)
            # elif c != cn:
            #     # np.arctan2(-y_lidar, x_lidar)
            #     res0 = np.arctan2(projected[r, c, 1] ,  projected[r, c, 0] )
            #     res1 = np.arctan2(projected[rn, cn, 1] ,  projected[rn, cn, 0] )
            #     res = abs(res0 - res1)


            theta = np.arctan2(d2 * np.sin(res), d1 - d2 * np.cos(res))
            if theta > angle_tollerance:
                Q.append((rn, cn))




def neighbourhood(r, c):
    # 某个点的上下左右四个点的坐标
    return ( ( r-1, c ),
             ( r, c-1 ),
             ( r+1, c ),
             ( r, c+1 )  )


if __name__ == "__main__":
    print('---------'*10)

    path = r'000017.bin'
    points = np.fromfile(path, dtype=np.float32).reshape(-1, 4)

    print('-------------------ransac----------------------------')
    s = time.time()
    indices, model = my_ransac_v2(points[:, :3],
                                 distance_threshold=0.3,)
    print('Time take my_ransac_v2:', time.time() - s)

    # points project to a image 2049*65*5
    projected_map = lidar_to_2d_front_view_depth(points, ground_indices=indices)
    print('Time take lidar_to_2d_front_view_depth:', time.time() - s)


    # 障碍物聚类，最主要的步骤
    labels = depth_clustering(projected_map)
    print('Time take depth_clustering:', time.time() - s)


    # pcl_visualization 可视化
    # 先得到一个65*2049*4 --> N*4
    color_labels = 10000 + labels*3000  # 区分颜色
    color_map = projected_map[:,:,:4]
    color_map[:, :, 3] = color_labels
    color_map = np.resize(color_map, (-1, 4))

    # 删除地面显示 delete ground
    # color_map = color_map[color_map[:, 3] > 10000, :]

    color_cloud = pcl.PointCloud_PointXYZRGB(color_map.astype(np.float32))
    visual = pcl.pcl_visualization.CloudViewing()
    visual.ShowColorCloud(color_cloud, b'cloud')
    flag = True
    while flag:
        flag != visual.WasStopped()





