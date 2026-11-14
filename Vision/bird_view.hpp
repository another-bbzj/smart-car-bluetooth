#pragma once

#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <limits>
#include <iostream>
#include <algorithm>
#include "track_config.hpp"

#include "calibration/UndistInverseMapH.txt"
#include "calibration/UndistInverseMapW.txt"
//本头文件中有一些鸟瞰图处理的函数，包括逆透视、绘制边线、平滑、重采样、寻找中线、寻找预瞄点、计算预瞄角度、计算转角度数、寻找角点和检测十字路口
namespace bird {

    constexpr int kInputWidth  = config::kInputWidth;
    constexpr int kInputHeight = config::kInputHeight;

    constexpr int kWidth  = 640;
    constexpr int kHeight = 480;

    // 原始图像边线点 -> 去畸变后的鸟瞰坐标。
    inline void transformPoints(
        const std::vector<cv::Point>& src,
        std::vector<cv::Point2f>& dst)
    {
        dst.clear();
        dst.reserve(src.size());

        for (const cv::Point& p : src) {
            if (p.x < 0 || p.x >= kInputWidth ||
                p.y < 0 || p.y >= kInputHeight) {
                continue;
            }

            const float x = UndistInverseMapW[p.y][p.x];
            const float y = UndistInverseMapH[p.y][p.x];

            if (!std::isfinite(x) || !std::isfinite(y)) {
                continue;
            }

            dst.emplace_back(x, y);
        }
    }

    // 这里只显示边线，不生成鸟瞰背景图。
    inline void drawPoints(
        cv::Mat& canvas,
        const std::vector<cv::Point2f>& points,
        const cv::Scalar& color)
    {
        for (const cv::Point2f& p : points) {
            // 显示时才筛掉画布外的点。
            if (p.x < 0 || p.x >= canvas.cols ||
                p.y < 0 || p.y >= canvas.rows) {
                continue;
            }
            cv::circle(canvas,
                cv::Point(cvRound(p.x), cvRound(p.y)),
                1, color, cv::FILLED);
        }
    }
    // 沿边线点序列做三角加权平滑。
    // radius = 2 时，窗口大小为 5，权重为 1、2、3、2、1。
    inline std::vector<cv::Point2f> smoothPoints(
        const std::vector<cv::Point2f>& src,int radius = 2)
    {
        if (src.empty() || radius <= 0) {
            return src;
        }

        const int count = static_cast<int>(src.size());
        std::vector<cv::Point2f> dst;
        dst.reserve(src.size());

        for (int i = 0; i < count; ++i) {
            cv::Point2f sum(0.f, 0.f);
            float weightSum = 0.f;

            for (int offset = -radius; offset <= radius; ++offset) {
                int index = i + offset;

                // 首尾不足一个窗口时，重复使用端点。
                if (index < 0) {
                    index = 0;
                } else if (index >= count) {
                    index = count - 1;
                }

                const float weight =
                    static_cast<float>(radius + 1 - std::abs(offset));

                sum += src[index] * weight;
                weightSum += weight;
            }

            dst.push_back(sum * (1.f / weightSum));
        }

        return dst;
    }
    //重采样函数
    inline std::vector<cv::Point2f> resamplePoints(
        const std::vector<cv::Point2f>& src,
        float spacing = 5.f)
    {
        std::vector<cv::Point2f> dst;

        if (src.empty() ||
            !std::isfinite(spacing) ||
            spacing <= 0.f) {
            return dst;
        }
        dst.push_back(src.front());
        // 从上一采样点开始，还需要走多少距离
        float remaining = spacing;

        for (std::size_t i = 1; i < src.size(); ++i) {
            cv::Point2f start = src[i - 1];
            const cv::Point2f end = src[i];
            float length = static_cast<float>(cv::norm(end - start));
            // 跳过重复点。
            if (length <= 1e-6f) {
                continue;
            }
            while (length >= remaining) {
                const float ratio = remaining / length;
                const cv::Point2f sample =
                    start + (end - start) * ratio;

                dst.push_back(sample);

                start = sample;
                length = static_cast<float>(cv::norm(end - start));
                remaining = spacing;
            }
            remaining -= length;
        }
        return dst;
    }
    // side = +1：从左边线向内偏移。
    // side = -1：从右边线向内偏移。
    // halfWidth：鸟瞰坐标中的道路半宽，单位为像素。
    inline std::vector<cv::Point2f> offsetToCenter(
        const std::vector<cv::Point2f>& edge,
        float halfWidth,
        int side)
    {
        std::vector<cv::Point2f> center;

        if (edge.size() < 2 || !std::isfinite(halfWidth) ||
            halfWidth <= 0.f ||(side != 1 && side != -1)) 
        {    return center;    }

        center.reserve(edge.size());
        const int count = static_cast<int>(edge.size());

        for (int i = 0; i < count; ++i) {
            // 用前后各两个采样点估计方向。
            const int before = (i >= 2) ? i - 2 : 0;
            const int after  = (i + 2 < count) ? i + 2 : count - 1;

            const cv::Point2f tangent = edge[after] - edge[before];
            const float length =
                static_cast<float>(cv::norm(tangent));

            if (length <= 1e-6f) {
                continue;
            }

            const float tx = tangent.x / length;
            const float ty = tangent.y / length;

            const cv::Point2f inward(
                -ty * static_cast<float>(side),
                tx * static_cast<float>(side));

            center.push_back(edge[i] + inward * halfWidth);
        }

        return center;
    }
    //寻找预瞄点函数
    inline bool findPreviewPoint(
        const std::vector<cv::Point2f>& center,
        cv::Point2f& vehiclePoint,
        cv::Point2f& previewPoint,
        float lookahead)
    {
        if (center.size() < 2 || !std::isfinite(lookahead) || lookahead <= 0.f) {
            return false;
        }

        // 当前先用鸟瞰图下方中点作为车辆参考点。
        vehiclePoint = cv::Point2f(300.f, 460.f);

        // 找距离车辆参考点最近的中线点。
        std::size_t nearest = 0;
        float nearestDistance = std::numeric_limits<float>::max();

        for (std::size_t i = 0; i < center.size(); ++i) {
            const cv::Point2f delta = center[i] - vehiclePoint;
            const float distance = delta.dot(delta);

            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearest = i;
            }
        }

        // 从最近点沿中线累计距离，寻找预瞄点。
        float accumulated = 0.f;

        for (std::size_t i = nearest + 1; i < center.size(); ++i) {
            accumulated += static_cast<float>(
                cv::norm(center[i] - center[i - 1]));

            if (accumulated >= lookahead) {
                previewPoint = center[i];

                // Per-frame diagnostics are written by the runner to CSV.
                return true;
            }
        }

        // 中线不够长时，使用末端点。
        previewPoint = center.back();
        return true;
    }
    // 车辆朝鸟瞰图上方行驶。
    // 返回角度单位：度；右偏为正，左偏为负。
    inline float calculatePreviewAngle(
        const cv::Point2f& vehiclePoint,
        const cv::Point2f& previewPoint)
    {
        const float dx = previewPoint.x - vehiclePoint.x;
        const float dy = vehiclePoint.y - previewPoint.y;

        return std::atan2(dx, dy) * 180.f /
            static_cast<float>(CV_PI);
    }

    inline float turnAngleDegrees(
        const std::vector<cv::Point2f>& points,
        std::size_t index,int window)
    {
        if (points.size() < 2 * static_cast<std::size_t>(window) + 1 ||
            index < static_cast<std::size_t>(window) ||
            index + static_cast<std::size_t>(window) >= points.size()) {
            return 0.f;
        }

        const cv::Point2f before =
            points[index] - points[index - window];

        const cv::Point2f after =
            points[index + window] - points[index];

        const float beforeLength = static_cast<float>(cv::norm(before));
        const float afterLength = static_cast<float>(cv::norm(after));

        if (beforeLength <= 1e-6f || afterLength <= 1e-6f) {
            return 0.f;
        }

        float cosine = before.dot(after) / (beforeLength * afterLength);

        // 防止浮点误差使 acos 输入略微超出 [-1, 1]。
        cosine = std::max(-1.f, std::min(1.f, cosine));

        return std::acos(cosine) * 180.f /
            static_cast<float>(CV_PI);
    }
    //找角点
    inline bool findCorner(const std::vector<cv::Point2f>& points,
        cv::Point2f& corner,float& bestAngle,int window = 4,
        float minAngle = 55.f,float maxAngle = 135.f)
    {
        if (points.size() < 2 * static_cast<std::size_t>(window) + 1) {
            return false;
        }

        bestAngle = 0.f;
        std::size_t bestIndex = 0;

        // 先只检查近端的一段。
        // 当前 sampleSpacing=5，100 个点大约对应 500 个鸟瞰像素。
        const std::size_t searchEnd = std::min(
            points.size() - static_cast<std::size_t>(window) - 1,
            static_cast<std::size_t>(100));

        for (std::size_t i = static_cast<std::size_t>(window);
            i <= searchEnd;++i) {
            const float angle = turnAngleDegrees(points, i, window);

            if (angle >= minAngle &&angle <= maxAngle &&angle > bestAngle) 
            {
                bestAngle = angle;
                bestIndex = i;
            }
        }

        if (bestAngle <= 0.f)
        {
            return false;
        }

        corner = points[bestIndex];
        return true;
    }
    //十字判定（左右都找到角点为十字）
    inline bool detectCross(const std::vector<cv::Point2f>& leftEdge,const std::vector<cv::Point2f>& rightEdge,
    cv::Point2f& leftCorner,cv::Point2f& rightCorner)
    {
        float leftAngle = 0.f;
        float rightAngle = 0.f;

        const bool leftFound = findCorner(leftEdge, leftCorner, leftAngle);

        const bool rightFound = findCorner(rightEdge, rightCorner, rightAngle);

        if (!leftFound || !rightFound) {
            return false;
        }

        // 左右边线的角点应该出现在相近纵向位置。
        if (std::abs(leftCorner.y - rightCorner.y) > 80.f) {
            return false;
        }

        return true;
    }

} // namespace bird