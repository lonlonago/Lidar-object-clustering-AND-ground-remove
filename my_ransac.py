

import numpy as np
import random



def my_ransac_v2(data,
              distance_threshold=0.3,
              P=0.99,
              sample_size=3,
              max_iterations=10000,
              ):
    """
    更改迭代条件
    加速计算
    :param data:
    :param sample_size:
    :param P :
    :param distance_threshold:
    :param max_iterations:
    :return:
    """
    # np.random.seed(12345)
    random.seed(12345)
    max_point_num = -999
    i = 0
    K = 10
    L_data = len(data)
    R_L = range(L_data)

    # for i in range(max_iterations):
    while i < K:
        # 随机选3个点  np.random.choice 很耗费时间，改为random模块
        # s3 = np.random.choice(L_data, sample_size, replace=False)
        # TODO 三个点的高度可以作为第一步初筛的标准;
        # TODO 2个点的y 距离可以作为第一步初筛的标准，避免出现把墙作为平面
        # 没有效果，一定要迭代固定的次数，才能找到平面，这是为什么，不是随机的吗？
        # if np.sum(data[s3,2] < 0.) != 3:
            # print(i,'point up ground. skip', data[s3,2])
            # continue
        # print('data[s3,:]',data[s3,2])
        # a = data[s3,2].copy()
        # a[a > 0] *= -1
        if abs(data[s3[0],1] - data[s3[1],1]) < 3:
            continue

        # 三个点的共线性检查; 改变了它的位置
        # if not check_in_one_line(data[s3,:]):
        #     print('error !  3 points in one line...')
        #     continue

        # 计算平面方程系数
        if coeffs is None:
            continue
        # 法向量的模, 如果系数标准化了就不需要除以法向量的模了
        r = np.sqrt(coeffs[0]**2 + coeffs[1]**2 + coeffs[2]**2 )
        # 计算每个点和平面的距离，根据阈值得到距离平面较近的点数量
        # d = np.abs(np.matmul(coeffs[:3], data.T) + coeffs[3]) / r
        d = np.divide(np.abs(np.matmul(coeffs[:3], data.T) + coeffs[3]) , r)
        # d = abs(np.matmul(coeffs[:3], data.T) + coeffs[3]) / r
        d_filt = np.array(d < distance_threshold)
        # d_filt = d < distance_threshold
        # d_filt = np.where(d < distance_threshold, 1,0)
        near_point_num = np.sum(d_filt,axis=0)
        # near_point_num = sum(d < distance_threshold)

        # print('coeffs:',coeffs,   )
        # print('coeffs2:', coeffs2/r,r ,coeffs2)
        # print('# inliers:', near_point_num, near_point_num2)

        if near_point_num > max_point_num:
            max_point_num = near_point_num
            # TODO 这是是否还有一个重新拟合模型的步骤，
            #  有的话怎么用多个点求一个平面
            best_model = coeffs
            best_filt = d_filt
            # if near_point_num > inliers_precent_threshold*len(data):
            #     break
            # 最大迭代次数K，是P的函数, 因为迭代次数足够多，就一定能找到最佳的模型，
            # P是模型提供理想需要结果的概率，也可以理解为模型的点都是内点的概率？
            #
            #
            # P = 0.99
            # 当前模型，所有点中随机抽取一个点，它是内点的概率
            w = near_point_num / L_data
            # np.power(w, 3)是随机抽取三个点，都是内点的概率
            # 1-np.power(w, 3)，就是三个点至少有一个是外点的概率，
            # 也就是得到一个坏模型的概率;
            # 1-P 代表模型永远不会选出一个3个点都是内点的集合的概率，
            # 也就是至少有一个外点；
            # K 就是得到的需要尝试多少次才会得到当前模型的理论次数
            # TODO 完善对该理论最大论迭代次数的理解
            wn = np.power(w, 3)
            p_no_outliers = 1.0 - wn
            # sd_w = np.sqrt(p_no_outliers) / wn
        # print('# K:', i, K, near_point_num)

        i += 1

        if i > max_iterations:
            print(' RANSAC reached the maximum number of trials.')
            break

    print('took iterations:', i+1, 'best model:', best_model,
          'explains:', max_point_num)
    return np.argwhere(best_filt).flatten(), best_model




def estimate_plane(xyz, normalize=True):
    """
    已知三个点，根据法向量求平面方程；
    返回平面方程的一般形式的四个系数
    :param xyz:  3*3 array
    x1 y1 z1
    x2 y2 z2
    x3 y3 z3
    :return: a b c d

      model_coefficients.resize (4);
      model_coefficients[0] = p1p0[1] * p2p0[2] - p1p0[2] * p2p0[1];
      model_coefficients[1] = p1p0[2] * p2p0[0] - p1p0[0] * p2p0[2];
      model_coefficients[2] = p1p0[0] * p2p0[1] - p1p0[1] * p2p0[0];
      model_coefficients[3] = 0;
      // Normalize
      model_coefficients.normalize ();
      // ... + d = 0
      model_coefficients[3] = -1 * (model_coefficients.template head<4>().dot (p0.matrix ()));

    """
    vector1 = xyz[1,:] - xyz[0,:]
    vector2 = xyz[2,:] - xyz[0,:]

    # 共线性检查
    if not np.all(vector1):
        print('will divide by zero..')
        return None
    dy1dy2 = vector2 / vector1
    # 2点如果是一条直线，那么必然它的xyz都是同一个比例关系
    if  not ((dy1dy2[0] != dy1dy2[1])  or  (dy1dy2[2] != dy1dy2[1])):
        return None


    b = (vector1[2]*vector2[0]) - (vector1[0]*vector2[2])
    c = (vector1[0]*vector2[1]) - (vector1[1]*vector2[0])
    # normalize
    if normalize:
        r = np.sqrt(a ** 2 + b ** 2 + c ** 2)
        a = a / r
        b = b / r
        c = c / r
    d = -(a*xyz[0,0] + b*xyz[0,1] + c*xyz[0,2])
    # return a,b,c,d
    return np.array([a,b,c,d])








