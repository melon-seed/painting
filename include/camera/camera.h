/**
 * @file camera.h
 * @author Cicada
 * @brief 
 * @date 2021-03-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

#include <opencv2/opencv.hpp>
#include "type.h"

namespace painting
{
    namespace camera
    {

class Camera
{
public:
    Camera() = default;
    virtual ~Camera();

    virtual bool Init() = 0;

    virtual void GetImage(cv::Mat* img) = 0;

    virtual void GetPointCloud(PointCloudPtr& clout_ptr) = 0;
};

    } // namespace camera
} // namespace painting
