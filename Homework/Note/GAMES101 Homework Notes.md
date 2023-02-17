# GAMES101 Homework Notes

---

*本笔记为个人(M1kanN)的学习笔记。记录用作复习*

---

[TOC]



## HomeWork_1

### 1. Reading the Doc of `Eigen`

[Eigen: Matrix and vector arithmetic](https://eigen.tuxfamily.org/dox/group__TutorialMatrixArithmetic.html)

*   定义向量和矩阵
    ```c++
    #include<eigen3/Eigen/Core>
    #include<eigen3/Eigen/Dense>
    
    Eigen::Matrix2cd mat;
    Eigen::Vector2d vec;
    Eigen::Vector2cd cvec
    ```

    `Matrix2cd`代表2x2的矩阵，并且元素均为`double`类型, 且为**复数**。`Vector2d`等等同理。

*   基本运算

    *   加、减、标量乘除
        Eigen重载了普通的符号。可以直接进行基本运算。

    *   向量/矩阵 乘法
        注意各个向量或者矩阵的运算顺序和大小，需要进行匹配才能运算。
        存放结果的变量的类型也需要匹配

    *   转置和共轭 （transpose and conjugate）
        ```c++
        Eigen::Matrix2d mat;
        mat.transpose();
        mat.conjugate();
        mat.adjoint();
        ```

        注意，`adjoint`并不是求伴随矩阵，而是求共轭转置，也就是厄尔米特矩阵。对于实数来说，跟`transpose`效果一致。

        *   原址变换问题（aliasing issue）
            在conjugate等方法中，并不会改变原来的变量的值。如果需要改变原来的变换值，参考下列例子，且不能将自己的转置单纯直接赋值给自己。
            也就是说，想原址变换 ，就用`InPlace()`更好。

            ```c++
            a = a.transpose();	// do NOT do this!
            a.transposeInPlace();
            ```

    *   点乘、叉乘
        注意叉乘只能用于3x3向量。

        ```c++
        a.dot(b);
        a.cross(b);
        ```

    *   辅助成员函数
        ```c++
        Eigen::Matrix2d mat;
        mat.sum();		// 参数和
        mat.prod();		// 所有参数乘
        mat.mean();		// 平均值
        mat.minCoeff();	// 参数最小值
        mat.maxCoeff();	// 参数最大值
        mat.trace();	// 迹
        ```

        

### 2. Coding

```c++
int main(){
    std::cout << "Homework 1 start here: " << std::endl;
    Eigen::Vector3d p(2.0f, 1.0f, 1.0f);   // homogeneous coordinate
    double theta = (45.0 / 180.0) * M_PI;   // 这里一定要用45.0 / 180.0，不然会直接向下取整！
    Eigen::Matrix3d r_marix;
    r_marix << cos(theta), -1.0 * sin(theta), 0,
                sin(theta), cos(theta), 0,
                0, 0, 1;
    Eigen::Matrix3d tr_matrix;
    tr_matrix << 1, 0, 1,
                0, 1, 2,
                0, 0, 1;
    // transform!
    Eigen::Vector3d result = tr_matrix * r_marix * p;
    std::cout << "Result of transforming: " << std::endl;
    std::cout << result << std::endl;

    return 0;
}
```

*   注意：
    犯了一个小错误：
    在计算theta的时候，用了`45 / 180`。这样得到的值是0！
    正确做法是：`45.0 / 180.0`



