# PointCloud_Ring_Judge
Determine whether the two-dimensional point cloud forms a ring
判断点云数据是否成环

# 问题描述
给定一系列的点（仅含x，y）信息，判断他们是否成环。

# 原理
假设有一系列的点<img src="https://latex.codecogs.com/svg.image?(x_i,y_i),i\in&space;1,2...n" title="(x_i,y_i),i\in 1,2...n" />，假设有一个初始的圆，其圆心为<img src="https://latex.codecogs.com/svg.image?(x_0,y_0)" title="(x_0,y_0)" />，其初始半径为R。

那么误差函数可以写为：

<img src="https://latex.codecogs.com/svg.image?Error&space;=&space;\frac{1}{n}\sum_{i=1}^n((x_i-x_0)^2&plus;(y_i-y_0)^2)" title="Error = \frac{1}{n}\sum_{i=1}^n((x_i-x_0)^2+(y_i-y_0)^2)" />

然后基于梯度下降法实现求解最优圆心位置和半径

初始圆心估计采用的是重心，初始半径是所有点与圆心距离的平均值

# 用法

见test.pp文件
