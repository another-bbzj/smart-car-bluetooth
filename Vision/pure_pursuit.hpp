#pragma once

#include <opencv2/core.hpp>
#include <cmath>

namespace control {

struct PurePursuitResult {
    bool valid = false;
    float lateralError = 0.f;       // 米，右侧为正
    float targetDistance = 0.f;     // 米，车辆到目标点的直线距离
    float curvature = 0.f;         // 1/米，右转为正
    float steeringDegrees = 0.f;   // 度，右转为正
};

inline PurePursuitResult calculatePurePursuit(const cv::Point2f& vehicle,
    const cv::Point2f& target, float metersPerPixel, float wheelBaseMeters)
{
    PurePursuitResult result;

    if (!std::isfinite(metersPerPixel) || metersPerPixel <= 0.f ||
        !std::isfinite(wheelBaseMeters) || wheelBaseMeters <= 0.f) {
        return result;
    }

    // 鸟瞰坐标：x 向右，y 向下；车辆前方为图像上方。
    const float lateral = (target.x - vehicle.x) * metersPerPixel;
    const float forward = (vehicle.y - target.y) * metersPerPixel;
    const float distanceSquared = lateral * lateral + forward * forward;

    if (!std::isfinite(distanceSquared) || forward <= 0.f || distanceSquared <= 1e-8f) {
        return result;
    }

    result.lateralError = lateral;
    result.targetDistance = std::sqrt(distanceSquared);
    result.curvature = 2.f * lateral / distanceSquared;
    result.steeringDegrees = std::atan(wheelBaseMeters * result.curvature)
        * 180.f / static_cast<float>(CV_PI);
    result.valid = true;

    return result;
}

} // namespace control