#pragma once

#include <cstddef>

namespace config {

// 输入图像尺寸
constexpr int kInputWidth = 640;
constexpr int kInputHeight = 480;

// 迷宫巡线
constexpr int kMazeBlockSize = 7;
constexpr int kMazeClipValue = 0;
constexpr int kMazeStartDirection = 0;
constexpr int kMazeStallLimit = 50;

// 鸟瞰边线处理
constexpr int kSmoothRadius = 2;
constexpr float kSampleSpacing = 5.f;
constexpr float kRoadHalfWidth = 96.f;

// 预瞄
constexpr float kLookahead = 80.f;
constexpr float kMetersPerPixel = 0.45/192.f;
constexpr float kWheelBaseMeters = 0.152f;
} // namespace config