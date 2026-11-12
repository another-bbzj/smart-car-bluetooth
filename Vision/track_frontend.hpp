#pragma once

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace track {

// OpenCV 8-bit HSV: H=0..179, S/V=0..255. Defaults are starting values,
// not thresholds calibrated on the competition venue.
struct FrontendConfig {
    int whiteMaxS = 85;
    int whiteMinV = 110;
    int blueMinH = 90;
    int blueMaxH = 135;
    int blueMinS = 70;
    int blueMinV = 30;
    int closeKernel = 3;        // 1 disables closing; use a small odd kernel.
    int maxHoleArea = 1500;    // fill only enclosed non-blue holes; 0 disables.
    double seedTopRatio = 0.55; // do not acquire distant track across a gap.
    double minRunRatio = 0.12;
    int seedMargin = 5;
    int blueProbe = 12;
    double minBlueRatio = 0.5;

    void validate() const {
        if (whiteMaxS < 0 || whiteMaxS > 255 || whiteMinV < 0 || whiteMinV > 255 ||
            blueMinH < 0 || blueMaxH > 179 || blueMinH > blueMaxH ||
            blueMinS < 0 || blueMinS > 255 || blueMinV < 0 || blueMinV > 255 ||
            closeKernel < 1 || closeKernel > 15 || closeKernel % 2 == 0 ||
            maxHoleArea < 0 || maxHoleArea > 10000 ||
            !std::isfinite(seedTopRatio) || seedTopRatio < 0.2 || seedTopRatio > 0.9 ||
            !std::isfinite(minRunRatio) || minRunRatio <= 0 || minRunRatio > 0.8 ||
            seedMargin < 2 || seedMargin > 30 || blueProbe < 1 || blueProbe > 50 ||
            !std::isfinite(minBlueRatio) || minBlueRatio <= 0 || minBlueRatio > 1) {
            throw std::invalid_argument("Invalid track frontend configuration");
        }
    }
};

inline FrontendConfig loadConfig(const std::string& path) {
    FrontendConfig c;
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) throw std::runtime_error("Cannot open config: " + path);
    auto read = [&](const char* name, auto& value) {
        const cv::FileNode node = fs[name];
        if (!node.empty()) node >> value;
    };
    read("white_max_s", c.whiteMaxS); read("white_min_v", c.whiteMinV);
    read("blue_min_h", c.blueMinH); read("blue_max_h", c.blueMaxH);
    read("blue_min_s", c.blueMinS); read("blue_min_v", c.blueMinV);
    read("close_kernel", c.closeKernel); read("max_hole_area", c.maxHoleArea);
    read("seed_top_ratio", c.seedTopRatio); read("min_run_ratio", c.minRunRatio);
    read("seed_margin", c.seedMargin); read("blue_probe", c.blueProbe);
    read("min_blue_ratio", c.minBlueRatio);
    c.validate();
    return c;
}

struct Observation {
    cv::Mat whiteMask; // raw white candidates
    cv::Mat blueMask;  // explicit blue exclusions
    cv::Mat roadMask;  // processed candidates, NOT a guaranteed drivable map
    bool seedFound = false;
    int seedY = -1;
    int leftX = -1;
    int rightX = -1;
    double blueSupport = 0; // weaker of the two side probes, NOT probability
};

inline void fillSmallNonBlueHoles(cv::Mat& mask, const cv::Mat& blue, int maxArea) {
    if (maxArea == 0) return;
    cv::Mat inverse, labels, stats, centroids;
    cv::bitwise_not(mask, inverse);
    const int count = cv::connectedComponentsWithStats(inverse, labels, stats, centroids, 8, CV_32S);
    std::vector<bool> allowed(count, false);
    for (int id = 1; id < count; ++id) {
        const int x = stats.at<int>(id, cv::CC_STAT_LEFT);
        const int y = stats.at<int>(id, cv::CC_STAT_TOP);
        const int w = stats.at<int>(id, cv::CC_STAT_WIDTH);
        const int h = stats.at<int>(id, cv::CC_STAT_HEIGHT);
        const int area = stats.at<int>(id, cv::CC_STAT_AREA);
        allowed[id] = area <= maxArea && x > 0 && y > 0 &&
            x + w < mask.cols && y + h < mask.rows;
    }
    // Never fill a component containing blue pixels (including blue gaps).
    for (int y = 0; y < mask.rows; ++y) {
        const int* ids = labels.ptr<int>(y);
        const uchar* b = blue.ptr<uchar>(y);
        for (int x = 0; x < mask.cols; ++x) if (b[x]) allowed[ids[x]] = false;
    }
    for (int y = 0; y < mask.rows; ++y) {
        const int* ids = labels.ptr<int>(y);
        uchar* dst = mask.ptr<uchar>(y);
        for (int x = 0; x < mask.cols; ++x) if (allowed[ids[x]]) dst[x] = 255;
    }
}

inline void findSeed(Observation& out, const FrontendConfig& c) {
    const int w = out.roadMask.cols, h = out.roadMask.rows;
    const int margin = std::max(c.seedMargin, c.blueProbe);
    const int minWidth = std::max(3, static_cast<int>(w * c.minRunRatio));
    if (w <= 2 * margin || h <= 2 * c.seedMargin) return;
    for (int y = h - 1 - c.seedMargin; y >= static_cast<int>(h * c.seedTopRatio); --y) {
        const uchar* row = out.roadMask.ptr<uchar>(y);
        const uchar* blue = out.blueMask.ptr<uchar>(y);
        double bestScore = -1e9;
        int bestL = -1, bestR = -1;
        double bestSupport = 0;
        for (int x = 0; x < w;) {
            if (!row[x]) { ++x; continue; }
            const int l = x;
            while (x < w && row[x]) ++x;
            const int r = x - 1;
            if (r - l + 1 < minWidth || l < margin || r >= w - margin) continue;
            int lb = 0, rb = 0;
            for (int k = 1; k <= c.blueProbe; ++k) {
                lb += blue[l - k] != 0; rb += blue[r + k] != 0;
            }
            const double support = static_cast<double>(std::min(lb, rb)) / c.blueProbe;
            if (support < c.minBlueRatio) continue;
            // Prefer the near, central road, not simply the widest bright object.
            const double score = (r - l + 1) - 0.5 * std::abs((l + r) * 0.5 - w * 0.5);
            if (score > bestScore) {
                bestScore = score; bestL = l; bestR = r; bestSupport = support;
            }
        }
        if (bestL >= 0) {
            out.seedFound = true; out.seedY = y; out.leftX = bestL; out.rightX = bestR;
            out.blueSupport = bestSupport;
            return;
        }
    }
}

inline Observation segment(const cv::Mat& bgr, const FrontendConfig& c) {
    c.validate();
    Observation out;
    if (bgr.empty()) return out;
    if (bgr.type() != CV_8UC3) throw std::invalid_argument("Frontend requires CV_8UC3 BGR input");
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 0, c.whiteMinV), cv::Scalar(179, c.whiteMaxS, 255), out.whiteMask);
    cv::inRange(hsv, cv::Scalar(c.blueMinH, c.blueMinS, c.blueMinV),
                cv::Scalar(c.blueMaxH, 255, 255), out.blueMask);
    out.roadMask = out.whiteMask.clone();
    if (c.closeKernel > 1) {
        cv::morphologyEx(out.roadMask, out.roadMask, cv::MORPH_CLOSE,
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(c.closeKernel, c.closeKernel)));
    }
    out.roadMask.setTo(0, out.blueMask);
    fillSmallNonBlueHoles(out.roadMask, out.blueMask, c.maxHoleArea);
    findSeed(out, c);
    return out;
}

} // namespace track
