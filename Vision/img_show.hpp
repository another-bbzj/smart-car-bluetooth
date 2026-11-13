#pragma once

// image_utils.hpp | OpenCV 4 + C++17 | 仅头文件（Header-Only）
// 用法：#include "image_utils.hpp"，调用 imgviz::drawPoints(image, points) 等函数。
// 所有函数的参数说明都写在函数正上方，格式采用 Doxygen 风格（@brief / @param / @return）。
// 颜色按 OpenCV 的 BGR 顺序填写：cv::Scalar(蓝, 绿, 红)。
// 注意：所有绘制函数会原地修改 image；灰度图会转换为三通道 BGR 图像。

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace imgviz {

using Segment = std::pair<cv::Point, cv::Point>;

/**
 * @brief 确保图像为 BGR 三通道，方便绘制彩色标记（原地操作）。
 * @param image [输入/输出] 待处理图像；1 通道灰度图转 BGR，3 通道不变，4 通道 BGRA 转 BGR。
 *              图像必须非空；转换后 image 可能引用新的图像数据。
 */
inline void ensureBGR(cv::Mat& image) {
    CV_Assert(!image.empty());
    switch (image.channels()) {
        case 1:
            cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
            break;
        case 3:
            break;
        case 4:
            cv::cvtColor(image, image, cv::COLOR_BGRA2BGR);
            break;
        default:
            CV_Error(cv::Error::StsBadArg, "Expected 1-, 3-, or 4-channel image");
    }
}

/**
 * @brief 生成 BGR 三通道图像的独立副本，不修改输入图像。
 * @param source [输入] 原图；支持 1、3、4 通道，不能为空。
 * @return cv::Mat 返回转换后的 BGR 图像副本。
 */
inline cv::Mat toBGR(const cv::Mat& source) {
    cv::Mat result = source.clone();
    ensureBGR(result);
    return result;
}

/**
 * @brief 从预定义调色板中获取一种 BGR 颜色（用于区分不同类别或组）。
 * @param index [输入] 颜色索引，从 0 开始；超过调色板长度会循环复用颜色。
 * @return cv::Scalar 选中的 BGR 颜色。
 */
inline cv::Scalar paletteColor(std::size_t index) {
    static const std::array<cv::Scalar, 10> palette = {{
        cv::Scalar(0, 255, 0),     // green
        cv::Scalar(0, 0, 255),     // red
        cv::Scalar(255, 0, 0),     // blue
        cv::Scalar(0, 255, 255),   // yellow
        cv::Scalar(255, 255, 0),   // cyan
        cv::Scalar(255, 0, 255),   // magenta
        cv::Scalar(0, 165, 255),   // orange
        cv::Scalar(180, 0, 180),   // purple
        cv::Scalar(255, 255, 255), // white
        cv::Scalar(160, 160, 0)    // teal
    }};
    return palette[index % palette.size()];
}

/**
 * @brief 绘制一条有限长度的直线段。
 * @param image     [输入/输出] 绘制目标图像，直接在原图上修改。
 * @param p1        [输入] 线段起点像素坐标 cv::Point(x1, y1)。
 * @param p2        [输入] 线段终点像素坐标 cv::Point(x2, y2)。
 * @param color     [输入] 线段颜色，BGR 顺序；默认 (0,255,0) 绿色。
 * @param thickness [输入] 线宽，单位为像素；默认 2。
 */
inline void drawLine(cv::Mat& image, cv::Point p1, cv::Point p2, const cv::Scalar& color = cv::Scalar(0, 255, 0), int thickness = 2)
{
    ensureBGR(image);
    cv::line(image, p1, p2, color, thickness, cv::LINE_AA);
}

/**
 * @brief 批量绘制线段：将 Vec4i 数组中的所有线段画成同一种颜色。
 * @param image     [输入/输出] 要绘制的图像。
 * @param lines     [输入] 线段数组，每个 cv::Vec4i={x1,y1,x2,y2}，兼容 HoughLinesP 的输出。
 * @param color     [输入] 所有线段统一使用的 BGR 颜色；默认绿色。
 * @param thickness [输入] 所有线段的线宽（像素）；默认 2。
 */
inline void drawLines(cv::Mat& image, const std::vector<cv::Vec4i>& lines, const cv::Scalar& color = cv::Scalar(0, 255, 0), int thickness = 2)
{
    ensureBGR(image);
    for (const cv::Vec4i& line : lines) {
        cv::line(image,
                 cv::Point(line[0], line[1]),
                 cv::Point(line[2], line[3]),
                 color, thickness, cv::LINE_AA);
    }
}

/**
 * @brief 批量绘制线段：使用“起点、终点成对”的线段数组。
 * @param image     [输入/输出] 要绘制的图像。
 * @param lines     [输入] 线段数组；每个 Segment 为 std::pair<cv::Point,cv::Point>。
 * @param color     [输入] 所有线段统一使用的 BGR 颜色；默认绿色。
 * @param thickness [输入] 线宽（像素）；默认 2。
 */
inline void drawLines(cv::Mat& image, const std::vector<Segment>& lines, const cv::Scalar& color = cv::Scalar(0, 255, 0), int thickness = 2)
{
    ensureBGR(image);
    for (const Segment& line : lines) {
        cv::line(image, line.first, line.second, color, thickness, cv::LINE_AA);
    }
}

/**
 * @brief 绘制多组线段：同组同色，不同组可以使用不同颜色。
 * @param image     [输入/输出] 要绘制的图像。
 * @param groups    [输入] 多组线段；groups[i] 是第 i 组 std::vector<cv::Vec4i>。
 * @param colors    [输入] 各组对应的 BGR 颜色数组；默认空，使用内置调色板；
 *                   若提供的颜色数量少于组数，会循环使用这些颜色。
 * @param thickness [输入] 各线段统一线宽（像素）；默认 2。
 */
inline void drawLineGroups(cv::Mat& image, const std::vector<std::vector<cv::Vec4i>>& groups, const std::vector<cv::Scalar>& colors = {}, int thickness = 2) 
{
    ensureBGR(image);
    for (std::size_t i = 0; i < groups.size(); ++i) {
        const cv::Scalar color = colors.empty()
            ? paletteColor(i) : colors[i % colors.size()];
        drawLines(image, groups[i], color, thickness);
    }
}

/**
 * @brief 绘制标准霍夫变换 HoughLines() 检出的直线（不是 HoughLinesP 的线段）。
 * @param image     [输入/输出] 要绘制的图像。
 * @param lines     [输入] cv::Vec2f 数组；每项={rho,theta}，rho 是距离（像素），
 *                   theta 是直线法向角（弧度），函数会换算成两个远端点再绘制。
 * @param color     [输入] BGR 线条颜色；默认绿色。
 * @param thickness [输入] 线宽（像素）；默认 2。
 */
inline void drawHoughLines(cv::Mat& image, const std::vector<cv::Vec2f>& lines, const cv::Scalar& color = cv::Scalar(0, 255, 0), int thickness = 2)
{
    ensureBGR(image);
    const double extent = 2.0 * std::max(image.cols, image.rows);
    for (const cv::Vec2f& line : lines) {
        const double rho = line[0];
        const double theta = line[1];
        const double a = std::cos(theta);
        const double b = std::sin(theta);
        const double x0 = a * rho;
        const double y0 = b * rho;
        const cv::Point p1(cvRound(x0 - b * extent), cvRound(y0 + a * extent));
        const cv::Point p2(cvRound(x0 + b * extent), cvRound(y0 - a * extent));
        cv::line(image, p1, p2, color, thickness, cv::LINE_AA);
    }
}

/**
 * @brief 【只画点，不连线】将 Point 数组中的每个点画成小圆点。
 * @param image  [输入/输出] 要绘制的图像。
 * @param points [输入] 点坐标数组 std::vector<cv::Point>；每项是像素坐标 (x,y)，
 *               无须排序，点与点之间不会连线。
 * @param color  [输入] 所有点使用的 BGR 颜色；默认 (0,0,255) 红色。
 * @param radius [输入] 每个点的圆形半径（像素，>=0）；默认 4。
 * @param filled [输入] 是否画实心圆；默认 true，设为 false 则画空心圆。
 */
inline void drawPoints(cv::Mat& image, const std::vector<cv::Point>& points, const cv::Scalar& color = cv::Scalar(0, 0, 255), int radius = 4, bool filled = true)
{
    ensureBGR(image);
    CV_Assert(radius >= 0);
    for (const cv::Point& p : points) {
        cv::circle(image, p, radius, color,
                   filled ? cv::FILLED : 2, cv::LINE_AA);
    }
}

/**
 * @brief 【只画点，不连线】绘制 Point2f 浮点坐标数组；绘制前四舍五入到像素坐标。
 * @param image  [输入/输出] 要绘制的图像。
 * @param points [输入] 浮点坐标数组 std::vector<cv::Point2f>，每项含 x、y。
 * @param color  [输入] 点的 BGR 颜色；默认红色。
 * @param radius [输入] 圆点半径（像素，>=0）；默认 4。
 * @param filled [输入] true 为实心圆、false 为空心圆；默认 true。
 */
inline void drawPoints(cv::Mat& image, const std::vector<cv::Point2f>& points, const cv::Scalar& color = cv::Scalar(0, 0, 255), int radius = 4, bool filled = true)
{
    ensureBGR(image);
    CV_Assert(radius >= 0);
    for (const cv::Point2f& p : points) {
        const cv::Point pixel(cvRound(p.x), cvRound(p.y));
        cv::circle(image, pixel, radius, color,
                   filled ? cv::FILLED : 2, cv::LINE_AA);
    }
}

/**
 * @brief 【点连成折线】按 Point 数组给定顺序，依次连接相邻的点。
 * @param image     [输入/输出] 要绘制的图像。
 * @param points    [输入] 有序点数组 std::vector<cv::Point>；空数组不绘制，
 *                   单个点画为圆点；多个点用直线段依次连接。
 * @param color     [输入] 折线的 BGR 颜色；默认 (0,255,255) 黄色。
 * @param thickness [输入] 线宽（像素）；默认 2。
 * @param closed    [输入] 是否将最后一点连接回第一点；默认 false（不闭合）。
 */
inline void drawPolyline(cv::Mat& image,
                         const std::vector<cv::Point>& points,
                         const cv::Scalar& color = cv::Scalar(0, 255, 255),
                         int thickness = 2,
                         bool closed = false) {
    ensureBGR(image);
    if (points.empty()) return;
    if (points.size() == 1) {
        cv::circle(image, points.front(), std::max(1, thickness / 2),
                   color, cv::FILLED, cv::LINE_AA);
        return;
    }
    cv::polylines(image, std::vector<std::vector<cv::Point>>{points},
                  closed, color, thickness, cv::LINE_AA);
}

/**
 * @brief 【点连成折线】Point2f 浮点坐标版本，绘制前四舍五入至整数像素。
 * @param image     [输入/输出] 要绘制的图像。
 * @param points    [输入] 有序点数组 std::vector<cv::Point2f>。
 * @param color     [输入] 折线 BGR 颜色；默认黄色。
 * @param thickness [输入] 线宽（像素）；默认 2。
 * @param closed    [输入] 是否首尾闭合；默认 false。
 */
inline void drawPolyline(cv::Mat& image, const std::vector<cv::Point2f>& points, const cv::Scalar& color = cv::Scalar(0, 255, 255), int thickness = 2, bool closed = false)
{
    std::vector<cv::Point> rounded;
    rounded.reserve(points.size());
    for (const auto& p : points)
        rounded.emplace_back(cvRound(p.x), cvRound(p.y));
    drawPolyline(image, rounded, color, thickness, closed);
}

/**
 * @brief 对有序点数组执行 Catmull-Rom 三次样条插值，生成平滑曲线的点序列。
 * @param points            [输入] 依次排列的控制点 std::vector<cv::Point2f>；
 *                          曲线经过这些点，但急转弯处可能产生过冲。
 * @param samplesPerSegment [输入] 每对相邻控制点之间生成的采样数（>=1）；默认 16，
 *                          数值越大绘制越细密（不是越准确）。
 * @param closed            [输入] 是否生成闭合曲线；默认 false。
 * @return std::vector<cv::Point2f> 曲线上的采样点，尚未绘制到图像上；
 *         非闭合曲线包含最后一个原始点，闭合曲线不重复首点。
 */
inline std::vector<cv::Point2f> catmullRomPoints( const std::vector<cv::Point2f>& points, int samplesPerSegment = 16,bool closed = false)
{
    CV_Assert(samplesPerSegment >= 1);
    if (points.size() <= 1) return points;

    const int n = static_cast<int>(points.size());
    const int segments = closed ? n : (n - 1);
    std::vector<cv::Point2f> result;
    result.reserve(static_cast<std::size_t>(segments) * samplesPerSegment + 1);

    const auto at = [&](int index) -> const cv::Point2f& {
        if (closed) {
            index = (index % n + n) % n;
        } else {
            index = std::max(0, std::min(index, n - 1));
        }
        return points[static_cast<std::size_t>(index)];
    };

    for (int i = 0; i < segments; ++i) {
        const cv::Point2f& p0 = at(i - 1);
        const cv::Point2f& p1 = at(i);
        const cv::Point2f& p2 = at(i + 1);
        const cv::Point2f& p3 = at(i + 2);

        for (int j = 0; j < samplesPerSegment; ++j) {
            const float t = static_cast<float>(j) / samplesPerSegment;
            const float t2 = t * t;
            const float t3 = t2 * t;
            const auto interpolate = [&](float a, float b, float c, float d) {
                return 0.5f * ((2.0f * b) + (-a + c) * t
                    + (2.0f * a - 5.0f * b + 4.0f * c - d) * t2
                    + (-a + 3.0f * b - 3.0f * c + d) * t3);
            };
            result.emplace_back(interpolate(p0.x, p1.x, p2.x, p3.x),
                                interpolate(p0.y, p1.y, p2.y, p3.y));
        }
    }
    if (!closed) result.push_back(points.back());
    return result;
}

/**
 * @brief 【点连成平滑曲线】将 Point2f 数组插值为 Catmull-Rom 曲线并绘制。
 * @param image             [输入/输出] 要绘制的图像。
 * @param points            [输入] 有序控制点 std::vector<cv::Point2f>。
 * @param color             [输入] 曲线 BGR 颜色；默认 (255,0,0) 蓝色。
 * @param thickness         [输入] 曲线线宽（像素）；默认 2。
 * @param samplesPerSegment [输入] 每两个相邻控制点间插值采样数（>=1）；默认 16。
 * @param closed            [输入] 是否首尾闭合；默认 false。
 * @param markControlPoints [输入] 是否额外用橙色小圆点显示原始控制点；默认 false。
 */
inline void drawSmoothCurve(cv::Mat& image,
                            const std::vector<cv::Point2f>& points,
                            const cv::Scalar& color = cv::Scalar(255, 0, 0),
                            int thickness = 2,
                            int samplesPerSegment = 16,
                            bool closed = false,
                            bool markControlPoints = false) {
    const auto smooth = catmullRomPoints(points, samplesPerSegment, closed);
    drawPolyline(image, smooth, color, thickness, closed);
    if (markControlPoints) {
        ensureBGR(image);
        for (const auto& p : points) {
            cv::circle(image, cv::Point(cvRound(p.x), cvRound(p.y)), 3,
                       cv::Scalar(0, 165, 255), cv::FILLED, cv::LINE_AA);
        }
    }
}

/**
 * @brief 【点连成平滑曲线】Point 整数坐标版本；内部先转换为 Point2f。
 * @param image             [输入/输出] 要绘制的图像。
 * @param points            [输入] 有序控制点 std::vector<cv::Point>。
 * @param color             [输入] 曲线 BGR 颜色；默认蓝色。
 * @param thickness         [输入] 曲线线宽（像素）；默认 2。
 * @param samplesPerSegment [输入] 每段插值采样数（>=1）；默认 16。
 * @param closed            [输入] 是否首尾闭合；默认 false。
 * @param markControlPoints [输入] 是否同时画出原始点；默认 false。
 */
inline void drawSmoothCurve(cv::Mat& image,
                            const std::vector<cv::Point>& points,
                            const cv::Scalar& color = cv::Scalar(255, 0, 0),
                            int thickness = 2,
                            int samplesPerSegment = 16,
                            bool closed = false,
                            bool markControlPoints = false) {
    std::vector<cv::Point2f> floatPoints;
    floatPoints.reserve(points.size());
    for (const auto& p : points)
        floatPoints.emplace_back(static_cast<float>(p.x),
                                 static_cast<float>(p.y));
    drawSmoothCurve(image, floatPoints, color, thickness,
                    samplesPerSegment, closed, markControlPoints);
}

/**
 * @brief 一次性绘制多个矩形框，可用于 ROI 和目标检测框可视化。
 * @param image      [输入/输出] 要绘制的图像。
 * @param rectangles [输入] 矩形数组 std::vector<cv::Rect>，每项含 x、y、width、height。
 * @param color      [输入] 边框 BGR 颜色；默认绿色。
 * @param thickness  [输入] 边框线宽（像素）；默认 2。
 */
inline void drawRectangles(cv::Mat& image, const std::vector<cv::Rect>& rectangles, const cv::Scalar& color = cv::Scalar(0, 255, 0), int thickness = 2)
{
    ensureBGR(image);
    for (const cv::Rect& rect : rectangles) {
        cv::rectangle(image, rect, color, thickness, cv::LINE_AA);
    }
}

/**
 * @brief 批量绘制轮廓点数组（例如 cv::findContours 的输出）。
 * @param image     [输入/输出] 要绘制的图像。
 * @param contours  [输入] 多条轮廓 std::vector<std::vector<cv::Point>>；每个子数组是一条轮廓。
 * @param color     [输入] 所有轮廓统一使用的 BGR 颜色；默认绿色。
 * @param thickness [输入] 轮廓线宽（像素）；默认 2；设为 cv::FILLED（-1）时填充轮廓。
 */
inline void drawContours(cv::Mat& image, const std::vector<std::vector<cv::Point>>& contours, const cv::Scalar& color = cv::Scalar(0, 255, 0), int thickness = 2)
{
    ensureBGR(image);
    if (contours.empty()) return;
    cv::drawContours(image, contours, -1, color, thickness, cv::LINE_AA);
}

/**
 * @brief 绘制圆检测结果，兼容 cv::HoughCircles() 返回的数据格式。
 * @param image      [输入/输出] 要绘制的图像。
 * @param circles    [输入] 圆数组 std::vector<cv::Vec3f>；每项={centerX,centerY,radius}，
 *                   均为像素单位，绘图前转成整数；负半径会被忽略。
 * @param color      [输入] 圆及圆心的 BGR 颜色；默认绿色。
 * @param thickness  [输入] 圆周线宽（像素）；默认 2。
 * @param markCenter [输入] 是否在每个圆心额外画小实心点；默认 true。
 */
inline void drawCircles(cv::Mat& image,
                        const std::vector<cv::Vec3f>& circles,
                        const cv::Scalar& color = cv::Scalar(0, 255, 0),
                        int thickness = 2,
                        bool markCenter = true) {
    ensureBGR(image);
    for (const cv::Vec3f& circle : circles) {
        const cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
        const int radius = cvRound(circle[2]);
        if (radius < 0) continue;
        cv::circle(image, center, radius, color, thickness, cv::LINE_AA);
        if (markCenter) cv::circle(image, center, 3, color, cv::FILLED, cv::LINE_AA);
    }
}

/**
 * @brief 在图像上绘制等间距的横向、纵向像素网格（用于调试坐标）。
 * @param image     [输入/输出] 要绘制的图像。
 * @param step      [输入] 相邻两条网格线的间距（像素，>0）；默认 50。
 * @param color     [输入] 网格线 BGR 颜色；默认 (80,80,80) 深灰色。
 * @param thickness [输入] 网格线宽（像素）；默认 1。
 */
inline void drawGrid(cv::Mat& image,
                     int step = 50,
                     const cv::Scalar& color = cv::Scalar(80, 80, 80),
                     int thickness = 1) {
    ensureBGR(image);
    CV_Assert(step > 0);
    for (int x = step; x < image.cols; x += step)
        cv::line(image, {x, 0}, {x, image.rows - 1}, color, thickness, cv::LINE_AA);
    for (int y = step; y < image.rows; y += step)
        cv::line(image, {0, y}, {image.cols - 1, y}, color, thickness, cv::LINE_AA);
}

} // namespace imgviz
