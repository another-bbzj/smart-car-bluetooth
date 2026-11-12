#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "bird_view.hpp"
#include "cross_state.hpp"
#include "pure_pursuit.hpp"
#include "ring_state.hpp"
//本文件中定义了调试绘制函数，包括绘制中线、预瞄点、十字路口状态、Pure Pursuit 结果和环形赛道状态
namespace debug_draw {

inline void drawCenterLines(cv::Mat& canvas, const std::vector<cv::Point2f>& centerL,
    const std::vector<cv::Point2f>& centerR)
{
    // 左侧推算中线：绿色
    bird::drawPoints(canvas, centerL, cv::Scalar(0, 255, 0));

    // 右侧推算中线：黄色
    bird::drawPoints(canvas, centerR, cv::Scalar(0, 255, 255));
}

inline void drawPreview(cv::Mat& canvas, bool hasPreview,
    const cv::Point2f& vehiclePoint, const cv::Point2f& previewPoint,
    float previewAngle)
{
    if (!hasPreview) {
        return;
    }

    cv::circle(canvas, cv::Point(cvRound(vehiclePoint.x), cvRound(vehiclePoint.y)),
        5, cv::Scalar(255, 255, 255), cv::FILLED);
    cv::circle(canvas, cv::Point(cvRound(previewPoint.x), cvRound(previewPoint.y)),
        6, cv::Scalar(0, 255, 255), cv::FILLED);
    cv::line(canvas, cv::Point(cvRound(vehiclePoint.x), cvRound(vehiclePoint.y)),
        cv::Point(cvRound(previewPoint.x), cvRound(previewPoint.y)), cv::Scalar(0, 255, 255), 2);
    cv::putText(canvas, cv::format("angle: %.1f deg", previewAngle),
        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
}

inline void drawCross(cv::Mat& canvas, bool crossFound,
    const cv::Point2f& leftCorner, const cv::Point2f& rightCorner)
{
    if (!crossFound) {
        return;
    }

    cv::circle(canvas, cv::Point(cvRound(leftCorner.x), cvRound(leftCorner.y)),
        7, cv::Scalar(0, 255, 0), cv::FILLED);
    cv::circle(canvas, cv::Point(cvRound(rightCorner.x), cvRound(rightCorner.y)),
        7, cv::Scalar(0, 255, 0), cv::FILLED);
    cv::putText(canvas, "CROSS", cv::Point(10, 60),
        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
}

inline void drawState(cv::Mat& canvas, bird::CrossState state)
{
    const char* text = "UNKNOWN";

    switch (state) {
    case bird::CrossState::None:
        text = "CROSS_NONE";
        break;
    case bird::CrossState::Begin:
        text = "CROSS_BEGIN";
        break;
    case bird::CrossState::In:
        text = "CROSS_IN";
        break;
    }

    cv::putText(canvas, text, cv::Point(10, 90),
        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
}
inline void drawPurePursuit(cv::Mat& canvas, const control::PurePursuitResult& pp,
    bool hasPreview, bool calibrated)
{
    cv::String text = "PP: uncalibrated";

    if (!hasPreview) {
        text = "PP: no preview";
    } else if (calibrated) {
        text = pp.valid
            ? cv::format("PP: k=%.3f  steer=%.1f deg", pp.curvature, pp.steeringDegrees)
            : "PP: invalid target";
    }

    cv::putText(canvas, text, cv::Point(10, 120),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
}
inline void drawRingState(cv::Mat& canvas, bird::RingState state)
{
    cv::putText(canvas, bird::ringStateName(state), cv::Point(10, 150),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 200, 0), 2);
}
} // namespace debug_draw