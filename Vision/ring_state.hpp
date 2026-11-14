#pragma once

#include <opencv2/opencv.hpp>
#include <cstddef>
#include "bird_view.hpp"
#include <algorithm>
namespace bird {

enum class RingState {
    None,
    LeftBegin,
    RightBegin,
    LeftIn,
    RightIn,
    LeftRunning,
    RightRunning,
    LeftOut,
    RightOut,
    LeftEnd,
    RightEnd
};
inline bool isLongStraight(const std::vector<cv::Point2f>& line)
{
    if (line.size() < 40) return false;

    const int window = 4;
    const std::size_t end = std::min(line.size() - window, std::size_t(40));
    float angleSum = 0.f;
    int count = 0, sharpCount = 0;

    for (std::size_t i = window; i < end; ++i) {
        const float angle = turnAngleDegrees(line, i, window);
        angleSum += angle;
        ++count;
        if (angle > 20.f) ++sharpCount;
    }

    return count >= 20 && angleSum / count <= 12.f && sharpCount <= count / 5;
}
inline bool detectLeftRingBegin(
    const std::vector<cv::Point2f>& leftLine,
    const std::vector<cv::Point2f>& rightLine,
    cv::Point2f& corner)
{
    float angle = 0.f;
    cv::Point2f otherCorner;
    const bool leftCorner = findCorner(leftLine, corner, angle);
    const bool rightCorner = findCorner(rightLine, otherCorner, angle);
    return leftCorner && !rightCorner && isLongStraight(rightLine);
}

inline bool detectRightRingBegin(
    const std::vector<cv::Point2f>& leftLine,
    const std::vector<cv::Point2f>& rightLine,
    cv::Point2f& corner)
{
    float angle = 0.f;
    cv::Point2f otherCorner;
    const bool rightCorner = findCorner(rightLine, corner, angle);
    const bool leftCorner = findCorner(leftLine, otherCorner, angle);
    return rightCorner && !leftCorner && isLongStraight(leftLine);
}

struct RingStateMachine {
    RingState state = RingState::None;
    int lostFrames = 0;
    int recoveredFrames = 0;
    int turnFrames = 0;
    int outFrames = 0;
    int entryFrames = 0;
    int entrySide = 0;
    RingState update(bool leftBegin, bool rightBegin, bool leftCornerFound, bool rightCornerFound,
    const std::vector<cv::Point2f>& leftLine, const std::vector<cv::Point2f>& rightLine)
    {
        const std::size_t leftCount = leftLine.size(), rightCount = rightLine.size();
        const bool leftLost = leftCount < 20, rightLost = rightCount < 20;

        // 点序应从近到远；比较近端约 50 像素弧长处的横向变化。
        const bool rightBendsLeft = rightCount > 10 &&
            rightLine[10].y < rightLine[0].y - 20.f && rightLine[10].x < rightLine[0].x - 10.f;
        const bool leftBendsRight = leftCount > 10 &&
            leftLine[10].y < leftLine[0].y - 20.f && leftLine[10].x > leftLine[0].x + 10.f;

        cv::Point2f leftExit, rightExit;
        float leftAngle = 0.f, rightAngle = 0.f;
        const bool leftExitNear = leftCornerFound && findCorner(leftLine, leftExit, leftAngle)
            && cv::norm(leftExit - leftLine.front()) < 130.f;
        const bool rightExitNear = rightCornerFound && findCorner(rightLine, rightExit, rightAngle)
            && cv::norm(rightExit - rightLine.front()) < 130.f;

        switch (state) {
        case RingState::None:
            lostFrames = recoveredFrames = turnFrames = outFrames = 0;
            {
                const int candidate = leftBegin && !rightBegin ? -1 : (rightBegin && !leftBegin ? 1 : 0);
                entryFrames = candidate != 0 && candidate == entrySide ? entryFrames + 1 : (candidate != 0 ? 1 : 0);
                entrySide = candidate;
                if (entryFrames >= 4) {
                    state = candidate == -1 ? RingState::LeftBegin : RingState::RightBegin;
                    entryFrames = entrySide = 0;
                }
            }
            break;

        case RingState::LeftBegin:
            if (leftLost) {
                if (lostFrames < 2) ++lostFrames;
                recoveredFrames = 0;
            } else if (lostFrames >= 2 && leftCount >= 40) {
                if (++recoveredFrames >= 2) {
                    state = RingState::LeftIn;
                    lostFrames = recoveredFrames = 0;
                }
            } else {
                recoveredFrames = 0;
                if (lostFrames < 2) lostFrames = 0;
            }
            break;

        case RingState::RightBegin:
            if (rightLost) {
                if (lostFrames < 2) ++lostFrames;
                recoveredFrames = 0;
            } else if (lostFrames >= 2 && rightCount >= 40) {
                if (++recoveredFrames >= 2) {
                    state = RingState::RightIn;
                    lostFrames = recoveredFrames = 0;
                }
            } else {
                recoveredFrames = 0;
                if (lostFrames < 2) lostFrames = 0;
            }
            break;

        case RingState::LeftIn:
            turnFrames = rightBendsLeft ? turnFrames + 1 : 0;
            if (turnFrames >= 3) {
                state = RingState::LeftRunning;
                turnFrames = 0;
            }
            break;

        case RingState::RightIn:
            turnFrames = leftBendsRight ? turnFrames + 1 : 0;
            if (turnFrames >= 3) {
                state = RingState::RightRunning;
                turnFrames = 0;
            }
            break;

        case RingState::LeftRunning:
            outFrames = rightExitNear ? outFrames + 1 : 0;
            if (outFrames >= 2) {
                state = RingState::LeftOut;
                outFrames = 0;
            }
            break;

        case RingState::RightRunning:
            outFrames = leftExitNear ? outFrames + 1 : 0;
            if (outFrames >= 2) {
                state = RingState::RightOut;
                outFrames = 0;
            }
            break;

        case RingState::LeftOut:
            recoveredFrames = isLongStraight(rightLine) ? recoveredFrames + 1 : 0;
            if (recoveredFrames >= 2) {
                state = RingState::LeftEnd;
                recoveredFrames = 0;
            }
            break;

        case RingState::RightOut:
            recoveredFrames = isLongStraight(leftLine) ? recoveredFrames + 1 : 0;
            if (recoveredFrames >= 2) {
                state = RingState::RightEnd;
                recoveredFrames = 0;
            }
            break;

        case RingState::LeftEnd:
        case RingState::RightEnd:
            // End 阶段继续沿外侧直线，内侧必须先丢失再恢复才能退出。
            {
            const bool innerLost = state == RingState::LeftEnd ? leftLost : rightLost;
            const bool innerRecovered = (state == RingState::LeftEnd ? leftCount : rightCount) >= 40;
            const bool outerStraight = isLongStraight(state == RingState::LeftEnd ? rightLine : leftLine);
            if (innerLost) {
                if (lostFrames < 2) ++lostFrames;
                recoveredFrames = 0;
            } else if (lostFrames >= 2 && innerRecovered && outerStraight) {
                ++recoveredFrames;
            } else {
                recoveredFrames = 0;
                if (lostFrames < 2) lostFrames = 0;
            }
            if (recoveredFrames >= 3) {
                state = RingState::None;
                lostFrames = recoveredFrames = turnFrames = outFrames = 0;
            }
            }
            break;
        }

        return state;
    }
};

inline const char* ringStateName(RingState state)
{
    switch (state) {
    case RingState::None: return "RING_NONE";
    case RingState::LeftBegin: return "LEFT_BEGIN";
    case RingState::RightBegin: return "RIGHT_BEGIN";
    case RingState::LeftIn: return "LEFT_IN";
    case RingState::RightIn: return "RIGHT_IN";
    case RingState::LeftRunning: return "LEFT_RUNNING";
    case RingState::RightRunning: return "RIGHT_RUNNING";
    case RingState::LeftOut: return "LEFT_OUT";
    case RingState::RightOut: return "RIGHT_OUT";
    case RingState::LeftEnd: return "LEFT_END";
    case RingState::RightEnd: return "RIGHT_END";
    }
    return "UNKNOWN";
}

} // namespace bird