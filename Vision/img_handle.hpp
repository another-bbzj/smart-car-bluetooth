#pragma once
#include <opencv2/opencv.hpp>
#include "track_frontend.hpp"
#include "img_show.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>
#include "TcpImshowServer.hpp"
#include "bird_view.hpp"
#include "cross_state.hpp"
#include "center_select.hpp"
#include "track_config.hpp"
#include "debug_draw.hpp"
#include "pure_pursuit.hpp"
#include "ring_state.hpp"
#include "global.hpp"
// 方向：0=上，1=右，2=下，3=左
static const int kDir[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};

// 取当前点附近 blockSize × blockSize 的平均亮度，作为局部判定依据。
// 局部阈值 = 平均亮度 - clipValue。
// 像素亮度低于局部阈值时视为墙；效果仍需结合光照验证。
// 迷宫巡线：side=+1 贴左墙，side=-1 贴右墙。
inline std::vector<cv::Point> mazeWalk(const cv::Mat& img, int startX, int startY, int side,
                                     bool binaryInput = false)
{   
    const int blockSize  = config::kMazeBlockSize;    // 局部阈值窗口，必须为大于 1 的奇数
    const int clipValue  = config::kMazeClipValue;   // 阈值偏移；越大越不容易判为墙
    const int startDir   = config::kMazeStartDirection;    // 初始方向，0 表示向上
    const int stallLimit = config::kMazeStallLimit;   // 连续若干步未向画面上方推进时退出

    std::vector<cv::Point> pts;
    if (img.empty() || img.type() != CV_8UC1) return pts;
    if (blockSize <= 1 || blockSize % 2 == 0) return pts;
    if (side != 1 && side != -1) return pts;

    const int maxPts = img.rows * 2;
    const int half   = blockSize / 2;
    const int W = img.cols, H = img.rows;

    // 左手：斜前方 = 正前 + 左(d+3)，撞墙右转(d+1)
    // 右手：斜前方 = 正前 + 右(d+1)，撞墙左转(d+3)
    const int diagTurn  = (side > 0) ? 3 : 1;
    const int blockTurn = (side > 0) ? 1 : 3;

    pts.reserve(maxPts);

    int x = startX, y = startY, dir = startDir, turn = 0;
    int bestY = y, sinceProgress = 0;

    while((int)pts.size() < maxPts && turn < 4 &&
           x > half && x < W - half - 1 &&
           y > half && y < H - half - 1)
    {
        if (sinceProgress > stallLimit) break;

        // Binary masks use an explicit threshold: a black neighborhood must
        // never become traversable just because its local average is zero.
        int localThres = 128;
        if (!binaryInput) {
            int sum = 0;
            for (int dy = -half; dy <= half; ++dy)
                for (int dx = -half; dx <= half; ++dx)
                    sum += img.at<uchar>(y + dy, x + dx);
            localThres = sum / (blockSize * blockSize) - clipValue;
        }

        const int fx = kDir[dir][0], fy = kDir[dir][1];
        const int diagDir = (dir + diagTurn) % 4;
        const int dx2 = kDir[dir][0] + kDir[diagDir][0];
        const int dy2 = kDir[dir][1] + kDir[diagDir][1];

        const int front = img.at<uchar>(y + fy, x + fx);
        const int diag  = img.at<uchar>(y + dy2, x + dx2);

        if (front < localThres) {
            // 正前方是墙：原地转向，不移动。
            dir = (dir + blockTurn) % 4;
            ++turn;
        } else if (diag < localThres) {
            // 正前方通、斜前方是墙：直走，贴着墙。
            x += fx;  y += fy;
            pts.emplace_back(x, y);
            turn = 0;
        } else {
            // 斜前方也通：斜走一步，并调整方向。
            x += dx2; y += dy2;
            dir = (dir + diagTurn) % 4;
            pts.emplace_back(x, y);
            turn = 0;
        }

        // 停滞保护：防止在丢线处反复摆动耗尽搜索步数。
        if (y < bestY) { bestY = y; sinceProgress = 0; }
        else           { ++sinceProgress; }
    }
    return pts;
}


// 寻找起始行。
// 从底部向上寻找第一个满足以下条件的候选行：
// 1. 有一段足够宽的连续亮区；
// 2. 亮区两端都在画面内，并保留 margin 边缘余量；
// 3. 两端外侧平均亮度小于赛道平均亮度的 wallRatio 倍。

inline bool findMazeStartRow(const cv::Mat& gray, int& seedY, int& leftX, int& rightX)
{
    const int margin      = 5;   // 边缘安全余量；调整巡线窗口时需同步检查
    const int minRunRatio = 6;   // 亮区宽度至少为图像宽度的 1/6
    const int probe       = 4;   // 亮区外侧探测像素数
    const double wallRatio = 0.55;   // 外侧像素与赛道平均亮度的比例上限

    if (gray.empty() || gray.type() != CV_8UC1) return false;

    const int W = gray.cols, H = gray.rows;
    const int minRun = W / minRunRatio;

    // 起始行保留底边余量，避免被巡线函数的边界条件拒绝。
    cv::Mat thresholdScratch; // 仅复用一行阈值输出，不生成整幅二值图。
    for (int y = H - margin; y >= H / 6; --y) {
        const int brightThres = static_cast<int>(cv::threshold(gray.row(y), thresholdScratch,
            0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU));
        const uchar* row = gray.ptr<uchar>(y);
        int bestLen = 0, bestL = -1, bestR = -1, runStart = -1;

        for (int x = 0; x < W; ++x) {
            const bool bright = row[x] > brightThres;
            if (bright && runStart < 0) runStart = x;
            if ((!bright || x == W - 1) && runStart >= 0) {
                const int end = bright ? x : x - 1;
                const int len = end - runStart + 1;
                if (len > bestLen) 
                { bestLen = len; bestL = runStart; bestR = end; }
                runStart = -1;
            }
        }

        if (bestLen < minRun) continue;
        if (bestL < margin || bestR > W - 1 - margin) continue;

        // 估计候选亮区的平均亮度。
        double trackSum = 0;
        int trackCnt = 0;
        for (int x = bestL; x <= bestR; x += 7) { trackSum += row[x]; ++trackCnt; }
        if (trackCnt == 0) continue;
        const double trackMean = trackSum / trackCnt;

        // 检查两端外侧是否足够暗，过滤亮度渐变等造成的假边界。
        double lSum = 0, rSum = 0;
        for (int k = 1; k <= probe; ++k) {
            lSum += row[bestL - k];
            rSum += row[bestR + k];
        }
        const double wallMax = trackMean * wallRatio;
        if (lSum / probe < wallMax && rSum / probe < wallMax) {
            seedY = y; leftX = bestL; rightX = bestR;
            return true;
        }
    }
    return false;
}

static bird::CrossStateMachine crossMachine;
static bird::CenterLineSelector centerSelector;
static bird::RingStateMachine ringMachine;
enum class TrackMode { Gray, Hsv };
struct ProcessOptions {
    TrackMode mode = TrackMode::Gray; // retain the old behavior for old callers
    track::FrontendConfig frontend;
};
struct FrameResult {
    bool seedFound = false;
    bool previewFound = false;
    int seedY = -1;
    double blueSupport = 0; // only defined for HSV mode; not confidence/probability
    std::size_t leftPoints = 0, rightPoints = 0;
    bird::CrossState cross = bird::CrossState::None;
    bird::RingState ring = bird::RingState::None;
    std::string selectedSide = "NONE";
    control::PurePursuitResult pursuit;
    cv::Mat mask, blueMask, overlay, birdImage;
};
inline void resetVisionState() {
    crossMachine = bird::CrossStateMachine{};
    centerSelector = bird::CenterLineSelector{};
    ringMachine = bird::RingStateMachine{};
}
inline FrameResult img_handle(const cv::Mat& img, bool saveBirdImage = false,
                              const ProcessOptions& options = ProcessOptions{}){
    FrameResult result;

    leftLine.clear();
    rightLine.clear();

    if (img.empty()) {
        resetVisionState();
        return result;
    }

    if (img.cols != bird::kInputWidth ||
        img.rows != bird::kInputHeight) {
        CV_Error(cv::Error::StsBadSize,"Calibration tables require a 640x480 input image");
    }
    // 灰度化。
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    // 高斯滤波。
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 1.4);
    // 起点使用逐行 OTSU，巡线使用局部阈值。
    // 以下固定阈值调用仅保留作对照，不参与运行。
    //cv::threshold(gray, binary, 200, 255, cv::THRESH_BINARY);

    // 起点搜索只计算当前候选行的阈值，不执行全图二值化。


    // 寻找左右边线。
    int seedY = -1, leftX = -1, rightX = -1;
    if (options.mode == TrackMode::Hsv) {
        const auto observation = track::segment(img, options.frontend);
        result.mask = observation.roadMask;
        result.blueMask = observation.blueMask;
        result.blueSupport = observation.blueSupport;
        if (observation.seedFound) {
            seedY = observation.seedY; leftX = observation.leftX; rightX = observation.rightX;
            leftLine = mazeWalk(result.mask, leftX + 1, seedY, +1, true);
            rightLine = mazeWalk(result.mask, rightX - 1, seedY, -1, true);
        }
    } else if (findMazeStartRow(blurred, seedY, leftX, rightX)) {
        leftLine  = mazeWalk(blurred, leftX + 1, seedY, +1);
        rightLine = mazeWalk(blurred, rightX - 1, seedY, -1);
    }
    result.seedFound = seedY >= 0;
    result.seedY = seedY;

    // 绘制原图坐标下的左右边线。
    // 单独建立 BGR 显示图，不在 blurred 上画点。
    // 保留 blurred 为单通道，供灰度巡线使用。
    cv::Mat vis;
    cv::cvtColor(blurred, vis, cv::COLOR_GRAY2BGR);
    // 标出实际选中的起始行和起点。
    if (seedY >= 0) {
        cv::line(vis,cv::Point(0, seedY),cv::Point(vis.cols - 1, seedY),cv::Scalar(0, 255, 255), 1);
        cv::circle(vis, cv::Point(leftX + 1, seedY),5, cv::Scalar(0, 255, 0), 2);
        cv::circle(vis, cv::Point(rightX - 1, seedY),5, cv::Scalar(0, 255, 255), 2);
    }
    imgviz::drawPoints(vis, leftLine,  cv::Scalar(0, 0, 255), 2);   // 红色：左边线
    imgviz::drawPoints(vis, rightLine, cv::Scalar(255, 0, 0), 2);   // 蓝色：右边线
    // 原图边线通过去畸变和逆透视组合表转换到鸟瞰坐标。
    std::vector<cv::Point2f> ipmL;
    std::vector<cv::Point2f> ipmR;

    bird::transformPoints(leftLine,  ipmL);
    bird::transformPoints(rightLine, ipmR);
    const auto smoothL = bird::smoothPoints(ipmL, config::kSmoothRadius);
    const auto smoothR = bird::smoothPoints(ipmR, config::kSmoothRadius);

    // 以鸟瞰像素为单位做等距重采样；物理距离需另行标定。
    const float sampleSpacing = config::kSampleSpacing;

    const auto sampleL = bird::resamplePoints(smoothL, sampleSpacing);

    const auto sampleR = bird::resamplePoints(smoothR, sampleSpacing);
    // 创建黑色鸟瞰显示画布。
    cv::Mat birdView(bird::kHeight,bird::kWidth,CV_8UC3,cv::Scalar(0, 0, 0));

    // 灰色显示平滑后的鸟瞰边线。
    bird::drawPoints(birdView, smoothL, cv::Scalar(100, 100, 100));
    bird::drawPoints(birdView, smoothR, cv::Scalar(100, 100, 100));

    // 红、蓝色显示等距重采样后的左右边线。
    bird::drawPoints(birdView, sampleL, cv::Scalar(0, 0, 255));
    bird::drawPoints(birdView, sampleR, cv::Scalar(255, 0, 0));
    const float roadHalfWidth = config::kRoadHalfWidth;
    const auto centerL = bird::offsetToCenter(sampleL, roadHalfWidth, +1);
    const auto centerR = bird::offsetToCenter(sampleR, roadHalfWidth, -1);
    cv::Point2f leftCorner(0.f, 0.f);
    cv::Point2f rightCorner(0.f, 0.f);

    const bool crossFound = bird::detectCross(
        sampleL, sampleR, leftCorner, rightCorner);
    cv::Point2f ringCorner(0.f, 0.f);
    const bool leftRingBegin = !crossFound &&
        bird::detectLeftRingBegin(sampleL, sampleR, ringCorner);

    const bool rightRingBegin = !crossFound &&
        bird::detectRightRingBegin(sampleL, sampleR, ringCorner);
    cv::Point2f ringLeftCorner(0.f, 0.f);
    cv::Point2f ringRightCorner(0.f, 0.f);
    float ringLeftAngle = 0.f;
    float ringRightAngle = 0.f;

    const bool ringLeftCornerFound =
        bird::findCorner(sampleL, ringLeftCorner, ringLeftAngle);

    const bool ringRightCornerFound =
        bird::findCorner(sampleR, ringRightCorner, ringRightAngle);
    bird::RingState ringState = ringMachine.state;

    const bool ringActive = ringState != bird::RingState::None;
    if (ringActive || (!crossFound && crossMachine.state == bird::CrossState::None)) {
        ringState = ringMachine.update(ringActive ? false : leftRingBegin,
            ringActive ? false : rightRingBegin, ringLeftCornerFound,
            ringRightCornerFound, sampleL, sampleR);
    } else {
        ringMachine.entryFrames = ringMachine.entrySide = 0;
    }
    // 车辆参考点：供状态机和预瞄计算使用。
    cv::Point2f vehiclePoint(300.f, 460.f);
    cv::Point2f previewPoint;

    // 更新十字状态；已激活的环岛屏蔽新的十字检测。
    const bird::CrossState state = crossMachine.update(!ringActive && crossFound, leftCorner,
        rightCorner, vehiclePoint, sampleL.size(), sampleR.size());

    bird::CenterLineSelection selected;

    if (state != bird::CrossState::None) {
        selected = centerSelector.update(state, centerL, centerR,
            vehiclePoint, sampleL.size(), sampleR.size());
    } else if (ringState != bird::RingState::None) {
        selected = bird::selectRingCenter(ringState, centerL, centerR);
    } else {
        selected = centerSelector.update(state, centerL, centerR,
            vehiclePoint, sampleL.size(), sampleR.size());
    }
    const std::vector<cv::Point2f>* selectedCenter =selected.line;

//    std::cout << "state=" << static_cast<int>(state)<< " selectedSide=" << selected.side
//           << " leftCount=" << sampleL.size()<< " rightCount=" << sampleR.size()<< std::endl;

    // 沿选中的中线寻找预瞄点。
    const float lookahead = config::kLookahead;
    bool hasPreview = false;

    if (selectedCenter != nullptr) {hasPreview = bird::findPreviewPoint
        (*selectedCenter,vehiclePoint,previewPoint,lookahead);
    }
    float previewAngle = 0.f;

    if (hasPreview) {previewAngle = bird::calculatePreviewAngle(
            vehiclePoint, previewPoint);
    }
    control::PurePursuitResult pp;

if (hasPreview) {
    pp = control::calculatePurePursuit(vehiclePoint, previewPoint,
        config::kMetersPerPixel, config::kWheelBaseMeters);
}
    debug_draw::drawCenterLines(birdView,centerL,centerR);
    if (selectedCenter != nullptr) {
        bird::drawPoints(birdView, *selectedCenter,cv::Scalar(255, 255, 255));
    }
    debug_draw::drawPreview(birdView,hasPreview,vehiclePoint,previewPoint,previewAngle);

    debug_draw::drawCross(birdView,crossFound,
    leftCorner,rightCorner);

    debug_draw::drawState(birdView,state);
    debug_draw::drawRingState(birdView, ringState);
    const bool ppCalibrated = config::kMetersPerPixel > 0.f && config::kWheelBaseMeters > 0.f;
    debug_draw::drawPurePursuit(birdView, pp, hasPreview, ppCalibrated);
    // 实时发送原图边线和鸟瞰调试画面。
    remote.imshow("maze", vis);
    remote.imshow("bird", birdView);
    if (!result.mask.empty()) remote.imshow("mask", result.mask);
    if (!result.blueMask.empty()) remote.imshow("blue", result.blueMask);
    if (saveBirdImage) cv::imwrite("out_bird.png", birdView);
    result.previewFound = hasPreview;
    result.leftPoints = sampleL.size(); result.rightPoints = sampleR.size();
    result.cross = state; result.ring = ringState;
    result.selectedSide = selected.side;
    result.pursuit = pp;
    result.overlay = vis; result.birdImage = birdView;
    return result;
}
//

//./test_run "DistortionImage.jpg"
//./test_run "newtest_crop.jpg"