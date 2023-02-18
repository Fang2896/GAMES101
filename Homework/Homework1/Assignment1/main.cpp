#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

constexpr double MY_PI = 3.1415926;

// Function: Get the Translation Matrix for moving to the eye_pos
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

/*
    TODO_1:
    @detail:
        逐个元素地构建模型变换矩阵，并返回该矩阵。
        此函数中，只需实现三维中绕z轴旋转的变换矩阵，不用处理平移和缩放
*/
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

/*
    TODO_2:
    @detail:
        使用给定的参数逐个元素地构建透视投影矩阵并返回该矩阵。
        frustum -> Cuboid -> regular cube(i.e. [-1, 1]^3)
*/
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

/*
    TODO_4:
    @detail:
        绕任意过原点的轴的旋转变换矩阵
        用Rodrigues' Rotation Formmula
*/
Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    double fangle = (angle / 180.0) * MY_PI;
    Eigen::Matrix4f rodrigues;
    Eigen::Matrix4f I = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f N;
    Eigen::Vector4f axi;
    Eigen::RowVector4f taxi;
    axi << axis.x(), axis.y(), axis.z(), 0;
    taxi << axis.x(), axis.y(), axis.z(), 0;
    N << 0, -axis.z(), axis.y(), 0,
        axis.z(), 0, -axis.x(), 0,
        -axis.y(), axis.x(), 0, 0,
        0, 0, 0, 1;
    rodrigues = cos(fangle) * I + (1 - cos(fangle)) * axi * taxi + sin(fangle) * N;
    rodrigues(3, 3) = 1;    // 非齐次坐标公式应用在齐次坐标上，需要往右下角改1
    return rodrigues;
}


/*
    TODO_3:
    @detail:
        自行补充你所需的其他操作
*/
int main(int argc, const char** argv)
{
    float angle = 0;
    float ra;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};
    // v0, v1, v2 -> screen coordinate
    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    /*
    自行输入
    */
    std::cout << "Please enter the axis and angle:" << std::endl;
    float rangle;   // 旋转角度
    Eigen::Vector3f raxis;  // 旋转轴
    std::cin >> raxis.x() >> raxis.y() >> raxis.z() >> ra;  //定义罗德里格斯旋转轴和角



    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }
    bool rflag = false;
    while (key != 27) { // 27的ASCII码对应着ESC键
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        /*
        增强功能
        */
        
        if (rflag) //如果按下r了，就开始绕给定任意轴旋转
            r.set_rodrigues(get_rotation(raxis, rangle));
        else
            r.set_rodrigues(get_rotation({ 0,0,1 }, 0));


        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        } else if (key == 'r') {
            rflag = true;
            rangle += ra;
        }
    }

    return 0;
}
