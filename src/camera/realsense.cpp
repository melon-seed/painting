#include "camera/realsense.h"

#include <algorithm>
#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>

namespace painting
{
    namespace camera
    {

bool Realsense::Init()
{
    using namespace rs2;
    pipeline_.start();
    return true;
}

void Realsense::GetImage(cv::Mat* img)
{
    rs2::frameset data = pipeline_.wait_for_frames();
    auto color = data.get_color_frame();
    cv::Mat image(cv::Size(color.get_width(), color.get_height()),
                  CV_8UC3,
                  (void*)color.get_data(),
                  cv::Mat::AUTO_STEP);
    *img = image.clone();
}

void Realsense::GetPointCloud(PointCloudPtr& clout_ptr)
{
    rs2::frameset data = pipeline_.wait_for_frames();
    auto color = data.get_color_frame();
    auto depth = data.get_depth_frame();
    rs2::pointcloud point_cloud;
    rs2::points points;
    point_cloud.map_to(color);
    points = point_cloud.calculate(depth);

}

void Realsense::ConvertToPclPointType(const rs2::points& points,
                                      const rs2::video_frame& color_frame,
                                      PointCloudPtr& cloud_ptr)
{
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud_ptr->width = static_cast<uint32_t>(sp.width());
    cloud_ptr->height = static_cast<uint32_t>(sp.height());
    cloud_ptr->is_dense = false;
    cloud_ptr->points.resize(points.size());
    auto texture_coord = points.get_texture_coordinates();
    auto vertex = points.get_vertices();

    int width = color_frame.get_width();
    int height = color_frame.get_height();
    const auto new_texture = reinterpret_cast<const uint8_t*>(color_frame.get_data());
    for (unsigned int i = 0; i < points.size(); ++i)
    {
        // position
        cloud_ptr->points[i].x = vertex[i].x;
        cloud_ptr->points[i].y = vertex[i].y;
        cloud_ptr->points[i].z = vertex[i].z;

        // texture
        int x_value = std::min(std::max(int(texture_coord[i].u * width + .5f), 0), width - 1);
        int y_value = std::min(std::max(int(texture_coord[i].v * height + .5f), 0), height - 1);
        int bytes = x_value *color_frame.get_bytes_per_pixel();
        int strides = y_value * color_frame.get_stride_in_bytes();
        int text_index = (bytes + strides);
        cloud_ptr->points[i].b = new_texture[text_index];
        cloud_ptr->points[i].g = new_texture[text_index + 1];
        cloud_ptr->points[i].r = new_texture[text_index + 2];
    }
}

void Realsense::GetRgbTexture(const rs2::video_frame& texture,
                              const rs2::texture_coordinate texture_xy)
{
    int width = texture.get_width();
    int height = texture.get_height();

    int x_value = std::min(std::max(int(texture_xy.u * width + .5f), 0), width - 1);
    int y_value = std::min(std::max(int(texture_xy.v * height + .5f), 0), height - 1);

    int bytes = x_value *texture.get_bytes_per_pixel();
    int strides = y_value * texture.get_stride_in_bytes();
    int text_index = (bytes + strides);

    const auto new_texture = reinterpret_cast<const uint8_t*>(texture.get_data());

    int nt1 = new_texture[text_index];
    int nt2 = new_texture[text_index + 1];
    int nt3 = new_texture[text_index + 2];

}

    } // namespace camera
} // namespace painting
