// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

float cross_2(Vector2f a, Vector2f b)
{
    return a.y() * b.x() - a.x() * b.y();
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t)
{
    auto v = t.toVector4();

    float minx = static_cast<float>(width);
    float miny = static_cast<float>(height);
    float maxx = 0;
    float maxy = 0;

    for(const auto& vert : v)
    {
        minx = std::min(minx, vert.x());
        maxx = std::max(maxx, vert.x());
        miny = std::min(miny, vert.y());
        maxy = std::max(maxy, vert.y());
    }
    
    int x0 = std::max(static_cast<int>(minx), 0);
    int x1 = std::min(static_cast<int>(maxx) + 1, width - 1);
    int y0 = std::max(static_cast<int>(miny), 0);
    int y1 = std::min(static_cast<int>(maxy) + 1, height - 1);

    auto getTriCoord = [](Vector2f p, Vector2f a, Vector2f b, Vector2f c)-> Vector3f
    {
        Vector2f V0 = b - a;
        Vector2f V1 = c - a;

        float s = cross_2(V0, V1);
        float bc = cross_2(b - p, c - p);
        float ca = cross_2(c - p, a - p);
        float ab = cross_2(a - p, b - p);

        return { bc / s, ca / s, ab / s };
    };

    Vector2f a = t.v[0].head<2>();
    Vector2f b = t.v[1].head<2>();
    Vector2f c = t.v[2].head<2>();

    Vector2f offset[4] = { {-1, -1}, {-1, 1}, {1, 1}, {1, -1} };
    Vector3f inverse_z( 1.0f / t.v[0].z(), 1.0f / t.v[1].z(), 1.0f / t.v[2].z());
    
    for(int x = x0; x <= x1; ++x)
    {
        for(int y = y0; y <= y1; ++y)
        {
            Vector2f center(static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f);
            Vector3f center_coord = getTriCoord(center, a, b, c);
            int index = get_index(x, y);
            float& buffer_z = depth_buf[index];
            float center_z = 1.0f / center_coord.dot(inverse_z);
            
            float alpha = 0.0f;
            for (int i = 0; i < 4; ++i)
            {
                Vector2f p = center + offset[i] * 0.25f;
                Vector3f coord = getTriCoord(p, a, b, c);
                if (coord.x() >= 0.0f && coord.y() >= 0.0f && coord.z() >= 0.0f)
                {
                    float z = 1.0f / coord.dot(inverse_z);
                    if(z >= buffer_z)
                        alpha += 0.25f;
                }
            }
            if (alpha > 0)
            {
                set_pixel(Vector3f(x, y, 0), t.getColor() * alpha + frame_buf[index] * (1 - alpha));
            }

            if(center_z >= buffer_z && center_coord.x() >= 0 && center_coord.y() >= 0 && center_coord.z() >= 0)
                buffer_z = center_z;
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), -std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on