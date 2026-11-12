// test_run.cpp —— 离线测试：用图片代替摄像头，跑一遍 img_handle
// 编译: g++ -std=c++17 -O2 test_run.cpp -o test_run $(pkg-config --cflags --libs opencv4) -pthread
// 运行: ./test_run [图片路径]

#include <opencv2/opencv.hpp>
#include <iostream>
#include "cross_state.hpp"
#include "TcpImshowServer.hpp"
#include "img_show.hpp"
#include "global.hpp"
#include "img_handle.hpp"
#include "pure_pursuit.hpp"
int main(int argc, char** argv)
{
    bird::CrossStateMachine machine;

const cv::Point2f vehicle(300.f, 450.f);
const cv::Point2f cornerFar(300.f, 300.f);  // 距离 150，大于 130
const cv::Point2f cornerNear(300.f, 400.f); // 距离 50，小于 130
// 仅用于验证公式的假设参数，不代表你的实车标定。
const cv::Point2f testVehicle(300.f, 450.f);
const cv::Point2f testTargets[] = {{300.f, 350.f}, {320.f, 350.f}, {280.f, 350.f}};

for (const auto& target : testTargets) {
    const auto pp = control::calculatePurePursuit(testVehicle, target, 0.005f, 0.15f);
    std::cout << "PP valid=" << pp.valid << " curvature=" << pp.curvature
              << " steering=" << pp.steeringDegrees << " deg\n";
}
struct TestFrame {
    bool crossFound;
    cv::Point2f corner;
    std::size_t leftCount;
    std::size_t rightCount;
};

const TestFrame frames[] = {
    {false, cornerFar, 50, 50}, // 普通状态
    {true,  cornerFar, 50, 50}, // 第一次发现
    {true,  cornerFar, 50, 50}, // 第二次发现，进入 Begin
    {false, cornerFar, 50, 50}, // Begin 中丢失第 1 帧
    {false, cornerFar, 50, 50}, // Begin 中丢失第 2 帧
    {true,  cornerNear, 50, 50},// 恢复且角点靠近，进入 In
    {false, cornerFar, 10, 10}, // In 中两侧变短，第 1 帧
    {false, cornerFar, 10, 10}, // In 中两侧变短，第 2 帧
    {false, cornerFar, 30, 30}, // 恢复第 1 帧
    {false, cornerFar, 30, 30}, // 恢复第 2 帧
    {false, cornerFar, 30, 30}, // 恢复第 3 帧，回到 None

};

for (std::size_t i = 0; i < sizeof(frames) / sizeof(frames[0]); ++i) {
    const auto& f = frames[i];

    const auto state = machine.update(
        f.crossFound,
        f.corner,
        f.corner,
        vehicle,
        f.leftCount,
        f.rightCount);

    const char* name = "UNKNOWN";
    switch (state) {
    case bird::CrossState::None:
        name = "CROSS_NONE";
        break;
    case bird::CrossState::Begin:
        name = "CROSS_BEGIN";
        break;
    case bird::CrossState::In:
        name = "CROSS_IN";
        break;
    }

    std::cout << "frame " << i
              << ": crossFound=" << f.crossFound
              << ", state=" << name << '\n';
}
    const char* path = (argc > 1) ? argv[1] : "test_track.png";
    cv::Mat img = cv::imread(path);
    if (img.empty()) { std::cerr << "imread failed: " << path << "\n"; return 1; }

    std::cout << "读入 " << path << "  " << img.cols << "x" << img.rows << "\n";

    // 和 main.cpp 一样调用同一个处理函数
    img_handle(img, true); // Save bird image only in offline tests.

    std::cout << "leftLine  = " << leftLine.size() << " 点";
    if (!leftLine.empty())
        std::cout << "  y: " << leftLine.front().y << " -> " << leftLine.back().y;
    std::cout << "\n";

    std::cout << "rightLine = " << rightLine.size() << " 点";
    if (!rightLine.empty())
        std::cout << "  y: " << rightLine.front().y << " -> " << rightLine.back().y;
    std::cout << "\n";

    // 把结果画出来存盘，方便肉眼检查
    cv::Mat vis;
    cv::cvtColor(blurred, vis, cv::COLOR_GRAY2BGR);
    imgviz::drawPoints(vis, leftLine,  cv::Scalar(0, 0, 255), 2);
    imgviz::drawPoints(vis, rightLine, cv::Scalar(255, 0, 0), 2);
    cv::imwrite("out_maze.png", vis);
    std::cout << "结果已保存: out_maze.png\n";

    return 0;
}
