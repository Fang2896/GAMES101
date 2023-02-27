# GAMES101 Homework_4 Notes

---

*本笔记为个人(M1kanN)的学习笔记。记录用作复习*

---

[TOC]





## Homework_4

### 1. Details

*   实现贝塞尔曲线函数



### 2. Code

*   按照公式来就行
    ```c++
    cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
    {
        // TODO: Implement de Casteljau's algorithm
        auto &p_0 = control_points[0];
        auto &p_1 = control_points[1];
        auto &p_2 = control_points[2];
        auto &p_3 = control_points[3];
    
        auto p_01 = (1 - t) * p_0 + t * p_1;
        auto p_12 = (1 - t) * p_1 + t * p_2;
        auto p_23 = (1 - t) * p_2 + t * p_3;
    
        auto p_02 = (1 - t) * p_01 + t * p_12;
        auto p_13 = (1 - t) * p_12 + t * p_23;
    
        auto p_03 = (1 - t) * p_02 + t * p_13;
    
        return cv::Point2f(p_03);
    }
    
    void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
    {
        // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
        // recursive Bezier algorithm.
        for(double t = 0.0; t <= 1.0; t += 0.001) {
            auto point = recursive_bezier(control_points, t);
            window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
            const float x[4] = {0, 0, 0.5, -0.5};
            const float y[4] = {+0.5, -0.5, 0, 0};
            int x_now = round(point.x), y_now = round(point.y);
            float d = std::sqrt(std::pow(point.x - x_now, 2) + std::pow(point.y - y_now, 2));
            for(int i = 0; i < 4; i++) {
                float x_neibor = floor(point.x + x[i]);
                float y_neibor = floor(point.y + y[i]);
                if(x_neibor >= 0 && x_neibor < 700 && y_neibor >= 0 && y_neibor < 700) {
                    float w = d / std::sqrt((std::pow(x_neibor - point.x, 2) + std::pow(y_neibor - point.y, 2)));
                    window.at<cv::Vec3b>(y_neibor, x_neibor)[1] = std::max(float(window.at<cv::Vec3b>(y_neibor, x_neibor)[1]), 255 * w);
                }
            }
        }
    }
    ```

### 3. Advance

参考：[Games101：作业4解析（含提高部分）](https://blog.csdn.net/Q_pril/article/details/123818346)

*   如何解决锯齿化？
    用模糊！
*   如何模糊？
    修改邻近几个点的RGB值即可
*   如何修改？取几个点？
    可以用距离之比来取。取多了会变粗。所以可以考虑取4个点。
    （联想一下作业2的提高部分）