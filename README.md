代码主体思想:因为在C++环境下使用mujoco的simulate环境配置起来非常复杂，因此在这里选择在C++中写完主要算法(MPC),之后在python进行可视化展示  
使用结构体是较好的能时间函数多输出的一种方式

1. 核心命名规则（理解后缀的秘密）
在 Eigen 中，矩阵和向量的类型名称通常遵循 Matrix/Vector + 尺寸 + 数据类型 的缩写规则：

数据类型：d (double), f (float), i (int), c (complex)。

尺寸：X (Dynamic，动态大小，运行时决定), 2, 3, 4 (固定大小，编译时决定)。

示例：

MatrixXd：动态大小的 double 类型矩阵（最常用）。

Matrix3f：3x3 的 float 类型矩阵。

VectorXd：动态大小的 double 类型列向量。

Vector3d：3维 double 类型列向量。