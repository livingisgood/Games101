#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Vector3f get_vec_rotation(const Vector3f& input, const Vector3f& axis, float cos_a, float sin_a)
{
    Vector3f proj = input.dot(axis) * axis;
    Vector3f offset = input - proj;
    Vector3f binormal = axis.cross(offset);
    return proj + cos_a * offset + sin_a * binormal;
}

Eigen::Matrix4f get_rotation(const Vector3f& axis, float angle)
{
    Vector3f normal = axis.normalized();
    if(normal.squaredNorm() < 0.9f)
        return Eigen::Matrix4f::Identity();

    angle = angle * 3.1415927f / 180.0f;
    float cos_a = cos(angle);
    float sin_a = sin(angle);

    Vector3f rotated_x = get_vec_rotation(Vector3f(1, 0, 0), normal, cos_a, sin_a);
    Vector3f rotated_y = get_vec_rotation(Vector3f(0, 1, 0), normal, cos_a, sin_a);
    Vector3f rotated_z = rotated_x.cross(rotated_y);

    Vector4f x(rotated_x.x(), rotated_x.y(), rotated_x.z(), 0.0f);
    Vector4f y(rotated_y.x(), rotated_y.y(), rotated_y.z(), 0.0f);
    Vector4f z(rotated_z.x(), rotated_z.y(), rotated_z.z(), 0.0f);
    
    Eigen::Matrix4f ret;
    ret.col(0) = x;
    ret.col(1) = y;
    ret.col(2) = z;
    ret.col(3) = Vector4f(0, 0, 0, 1);
    return ret;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    //return Eigen::Matrix4f::Identity();
    // Create the model matrix for rotating the triangle around the Z axis.
    return get_rotation(Vector3f(0, 0, 1), rotation_angle);    
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    Eigen::Matrix4f projection;

    float py = 1 / tan(eye_fov * 3.141593f / 180.f);
    float px = py / aspect_ratio;
    float k = zNear + zFar / (zFar - zNear);
    float b = 2 * zNear * zFar / (zFar - zNear);

    // camera looking towards -z, plane z = -near mapping to 1, z = -far mapping to -1.
    
    projection <<
        px, 0, 0, 0,
        0, py, 0, 0,
        0, 0, k, b,
        0, 0, -1, 0;
    
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

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

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1f, 50));

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
        }
    }

    return 0;
}
