/**
 * @file realsense.h
 * @author Cicada
 * @brief 
 * @date 2021-03-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <librealsense2/rs.hpp>
#include "camera.h"

namespace painting
{
    namespace camera
    {

class Realsense : public Camera
{
public:
    Realsense() = default;
    ~Realsense() override;

    bool Init() override;

    void GetImage(cv::Mat* img) override;

    void GetPointCloud(PointCloudPtr& clout_ptr) override;

private:
    rs2::pipeline pipeline_;

    void ConvertToPclPointType(const rs2::points& points,
                               const rs2::video_frame& texture,
                               PointCloudPtr& cloud_ptr);

    void GetRgbTexture(const rs2::video_frame& texture,
                       const rs2::texture_coordinate texture_xy);
};

    } // namespace camera
} // namespace painting
