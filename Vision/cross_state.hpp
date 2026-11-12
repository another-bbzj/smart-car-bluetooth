#pragma once

#include <opencv2/opencv.hpp>
#include <cstddef>
//本文件中定义了十字路口状态机的枚举类型和类，包括状态的更新逻辑
namespace bird {

enum class CrossState {
    None,   // CROSS_NONE：普通循迹
    Begin,  // CROSS_BEGIN：检测到十字角点
    In      // CROSS_IN：进入十字内部
};

struct CrossStateMachine {
    CrossState state = CrossState::None;

    int seenFrames = 0;
    int lostFrames = 0;
    int recoveredFrames = 0;
    int crossLostFrames = 0;
    bool sawBothLinesLost = false;

    CrossState update(bool crossFound, const cv::Point2f& leftCorner,
        const cv::Point2f& rightCorner, const cv::Point2f& vehiclePoint,
        std::size_t leftCount, std::size_t rightCount)
    {
        constexpr int confirmFrames = 2;
        constexpr float cornerNearPixels = 130.f;
        constexpr std::size_t shortLineCount = 20;
        constexpr int lostConfirmFrames = 2;
        constexpr int recoveredConfirmFrames = 3;

        const float leftDistance = static_cast<float>(cv::norm(leftCorner - vehiclePoint));
        const float rightDistance = static_cast<float>(cv::norm(rightCorner - vehiclePoint));

        const bool cornerNear =
            leftDistance < cornerNearPixels ||
            rightDistance < cornerNearPixels;

        const bool bothLinesShort =
            leftCount < shortLineCount &&
            rightCount < shortLineCount;

        switch (state) {
        case CrossState::None:
            crossLostFrames = 0;

            if (crossFound) {
                ++seenFrames;
            } else {
                seenFrames = 0;
            }

            if (seenFrames >= confirmFrames) {
                state = CrossState::Begin;
                crossLostFrames = 0;
            }
            break;

        case CrossState::Begin:
            if (crossFound) {
                crossLostFrames = 0;

                if (cornerNear || bothLinesShort) {
                    state = CrossState::In;
                }
            } else {
                ++crossLostFrames;

                if (crossLostFrames >= 4) {
                    seenFrames = 0;
                    crossLostFrames = 0;
                    state = CrossState::None;
                }
            }
            break;

        case CrossState::In:
            if (bothLinesShort) {
                ++lostFrames;
                recoveredFrames = 0;

                if (lostFrames >= lostConfirmFrames) {
                    sawBothLinesLost = true;
                }
            } else {
                lostFrames = 0;

                if (sawBothLinesLost &&
                    leftCount >= shortLineCount &&
                    rightCount >= shortLineCount) {
                    ++recoveredFrames;
                } else {
                    recoveredFrames = 0;
                }
            }

            if (sawBothLinesLost && recoveredFrames >= recoveredConfirmFrames) {
                state = CrossState::None;
                seenFrames = 0;
                lostFrames = 0;
                recoveredFrames = 0;
                sawBothLinesLost = false;
            }
            break;
        }

        return state;
    }
};

} // namespace bird