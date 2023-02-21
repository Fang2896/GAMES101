# GAMES101 Homework Notes

---

*本笔记为个人(M1kanN)的学习笔记。记录用作复习*

---

[TOC]



## HomeWork_0

### 1. Reading the Doc of `Eigen`

[Eigen: Matrix and vector arithmetic](https://eigen.tuxfamily.org/dox/group__TutorialMatrixArithmetic.html)

*   定义向量和矩阵
    ```c++
    #include<eigen3/Eigen/Core>
    #include<eigen3/Eigen/Dense>
    
    Eigen::Matrix2cd mat;
    Eigen::Vector2d vec;
    Eigen::Vector2cd cvec使用像素中
    心的屏幕空间坐标来检查中心点
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
        mat.tra做法是：45.0 / 180.0nspose();
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



## Homework_1

### 1. Details

*   配置环境：`Egien`与`opencv2`

*   构建模型矩阵
*   构建投影透视矩阵
*   提高：构建绕任意过原点的轴的旋转变换矩阵

### 2. Code

*   模型矩阵
    ```c++
    Eigen::Matrix4f get_model_matrix(float rotation_angle)
    {
        Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    
        // TODO: Implement this function
        // Create the model matrix for rotating the triangle around the Z axis.
        // Then return it.
    
        /*  view transformation:
        * 1. position <e>
        * 2. gaze direction <g>
        * 3. Up direction <t>
        */
        Eigen::Matrix4f rotation;
        double alpha = (rotation_angle / 180.0) * MY_PI;
        rotation << cos(alpha), -1.0 * sin(alpha), 0, 0,
                    sin(alpha), cos(alpha), 0, 0,
                    0 ,0, 1, 0,
                    0, 0, 0, 1;
        model = rotation * model;
    
        return model;
    }
    ```

*   投影矩阵
    ```c++
    Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                          float zNear, float zFar)
    {
        // Students will implement this function
    
        Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    
        // TODO: Implement this function
        // Create the projection matrix for the given parameters.
        // Then return it.
        
        /*  Perspective Projection:
        * 1. Squish (Pers->Orthographic)
        * 2. Orthographic Projection
        *   parameters:
        * eye_fov: field of view Y
        * aspect_ratio: width / height
        * 
        * Example: get_projection_matrix(45, 1, 0.1, 50)
        */
        double theta = ((eye_fov / 2) / 180.0) * MY_PI; // half angle of field of eye
        double yHeight = 2.0 * zNear * tan(theta);
        double xWidth = aspect_ratio * yHeight;
        double zLong = zNear - zFar;    // 这里注意一下：由于标准是看向-z轴，所以这里应该要near - far
        Eigen::Matrix4f squish;
        Eigen::Matrix4f ortho;
        squish << zNear, 0, 0, 0,
                    0, zNear, 0, 0,
                    0, 0, zNear + zFar, -1.0 * zNear * zFar,
                    0, 0, 1, 0;
        ortho << 2.0 / xWidth, 0, 0, 0,
                    0, 2.0 / yHeight, 0, 0,
                    0, 0, 2.0 / zLong, 0,
                    0, 0, 0, 1;
    
        projection = ortho * squish * projection;
    
        return projection;
    }
    ```

*   **倒三角形的原因：**
    *   实际上画出来的三角形是倒着的，是因为三角形落在了摄像头的后方。注意到`get_projection_matrix`函数穿进去的zNear和zFar都是正的，但是三角形的z是负的。所以落在了摄像头的后方。这时候用projection矩阵，就会有一个类似对称翻转的效果了
    *   参考：[倒三角形原因](https://zhuanlan.zhihu.com/p/453150407)
    *   解决方法： (距离和坐标分开算！)
        `get_projection_matrix`函数中先进行一次正负转换`zNear = -abs(zNear); zFar = -abs(zFar);`同时在计算top时保持正数`float t = tan((eye_fov / 2) / 180 * MY_PI) * abs(zNear);`因为这里的zNear指的是距离

### 3. Advance

*   提高项：
    在 main.cpp 中构造一个函数，该函数的作用是得到绕任意过原点的轴的旋转变换矩阵。
    *   思路：用Rodrigues旋转公式。
    *   步骤：
        *   在`main.cpp`添加`get_rotation()`函数，得到Rodrigues矩阵。
        *   在`main()`函数里，写出能让用户输出自己想要旋转的轴，以及角度，按R则绕着输入轴旋转输入的角度。
        *   在`while`循环中计算rodrigues矩阵。并输入到`set_rodrigues()`函数中
        *   添加按键控制。
        *   在`rasterizer.cpp`中，添加`set_rodrigues`函数。
        *   在`draw`函数中，修改mvp的计算，在最后增加乘以rodrigues矩阵。
        *   注意修改hpp文件的成员变量和成员函数。



## Homework_2

### 1. Details

*   目标：
    在屏幕上画出一个实心三角形。即：栅格化一个三角形。
*   细节：
    上一次作业中，在视口变化之后，我们调用了函数`rasterize_wireframe(const Triangle& t)`。此次，我们需要自己填写并调用函数`rasterize_triangle(const Triangle& t)`。
*   函数流程：
    1.   创建三角形的2维bounding box
    2.   遍历此bounding box内的所有像素（使用整数索引）。然后，使用像素中心的屏幕空间坐标来检查中心点是否在三角形内。
    3.   如果在内部，则将其位置处的插值深度值 (interpolated depth value) 与深度 缓冲区 (depth buffer) 中的相应值进行比较。
    4.   如果当前点更靠近相机，请设置像素颜色并更新深度缓冲区 (depth buffer)。（z-buffer算法）
*   需要修改的函数：
    *   `rasterize_triangle()`: 执行三角形栅格化算法
    *   `static bool insideTriangle()`: 测试点是否在三角形内。你可以修改此函 数的定义，这意味着，你可以按照自己的方式更新返回类型或函数参数
    *   `get_projection_matrix()`: 复制粘贴在第一次作业中的实现来填充此函数。

### 2. Code

*   判断点是否在三角形里面（包括边界）
    ```c++
    static bool insideTriangle(int x, int y, const Vector3f* _v)
    {   
        // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
        Vector3f v01 = {_v[1].x() - _v[0].x(), _v[1].y() - _v[0].y(), 0};
        Vector3f v12 = {_v[2].x() - _v[1].x(), _v[2].y() - _v[1].y(), 0};
        Vector3f v20 = {_v[0].x() - _v[2].x(), _v[0].y() - _v[2].y(), 0};
        
        Vector3f v0p = {x - _v[0].x(), y - _v[0].y(), 0};
        Vector3f v1p = {x - _v[1].x(), y - _v[1].y(), 0};
        Vector3f v2p = {x - _v[2].x(), y - _v[2].y(), 0};
    
        return (v01.cross(v0p).z() <= 0) && (v12.cross(v1p).z() <= 0) && (v20.cross(v2p).z() <= 0);
    }
    ```

    
