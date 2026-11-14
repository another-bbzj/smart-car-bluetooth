#include "img_handle.hpp"
#include <filesystem>
#include <iostream>
#include <stdexcept>

void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
    std::cout << "PASS: " << message << '\n';
}
cv::Mat road() {
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(180, 50, 20));
    cv::rectangle(image, cv::Rect(200, 0, 240, 480), cv::Scalar(220, 220, 220), cv::FILLED);
    return image;
}
int main(int argc, char** argv) {
    try {
        const track::FrontendConfig c;
        auto straight = road();
        auto a = track::segment(straight, c);
        check(a.seedFound && a.leftX == 200 && a.rightX == 439, "white road on blue background has expected seed");
        auto dim = straight.clone(); dim.convertTo(dim, -1, 0.65);
        check(track::segment(dim, c).seedFound, "moderately dim synthetic road still acquired");
        check(!track::segment(cv::Mat(480, 640, CV_8UC3, cv::Scalar(255,255,255)), c).seedFound,
              "all-white frame rejected without blue support");
        cv::Mat blue(480, 640, CV_8UC3, cv::Scalar(180,50,20));
        check(!track::segment(blue, c).seedFound, "all-blue frame rejected");
        auto grayBackground = road();
        grayBackground(cv::Rect(0,0,200,480)).setTo(cv::Scalar(20,20,20));
        grayBackground(cv::Rect(440,0,200,480)).setTo(cv::Scalar(20,20,20));
        check(!track::segment(grayBackground, c).seedFound, "HSV mode rejects dark non-blue background");
        for (const auto& color : {cv::Scalar(0,0,220), cv::Scalar(0,220,0), cv::Scalar(0,220,220)}) {
            auto obstacle = road(); cv::circle(obstacle, cv::Point(320,455), 14, color, cv::FILLED);
            auto o = track::segment(obstacle, c);
            check(o.seedFound && o.roadMask.at<uchar>(455,320) == 255,
                  "small enclosed colored hole does not split the road seed");
        }
        auto blueHole = road(); cv::circle(blueHole, {320,400}, 14, cv::Scalar(180,50,20), cv::FILLED);
        check(track::segment(blueHole, c).roadMask.at<uchar>(400,320) == 0, "blue hole never filled");
        auto gap = road();
        cv::rectangle(gap, cv::Rect(0,260,640,220), cv::Scalar(180,50,20), cv::FILLED);
        check(!track::segment(gap, c).seedFound, "distant white road beyond gap is not acquired as near road");
        auto narrowGap = road(); narrowGap.row(350).setTo(cv::Scalar(180,50,20));
        check(track::segment(narrowGap, c).roadMask.at<uchar>(350,320) == 0, "closing does not bridge blue strip");
        auto edgeRoad = road(); cv::rectangle(edgeRoad, cv::Rect(0,0,210,480), cv::Scalar(220,220,220), cv::FILLED);
        check(!track::segment(edgeRoad, c).seedFound, "road with clipped boundary is reported unavailable");
        check(!track::segment(cv::Mat(), c).seedFound, "empty input has no seed");
        bool rejected = false;
        try { auto invalid = c; invalid.closeKernel = 4; invalid.validate(); } catch (const std::invalid_argument&) { rejected = true; }
        check(rejected, "even morphology kernel rejected");
        auto contour = mazeWalk(a.roadMask, a.leftX + 1, a.seedY, 1, true);
        check(!contour.empty(), "binary boundary walker finds road boundary");
        bool inside = true;
        for (const auto& p : contour) inside = inside && a.roadMask.at<uchar>(p) != 0;
        check(inside, "binary boundary walker never steps onto blue background");
        check(mazeWalk(cv::Mat::zeros(480,640,CV_8UC1),320,450,1,true).empty(), "black mask does not become traversable");
        ProcessOptions options; options.mode = TrackMode::Hsv;
        resetVisionState();
        (void)img_handle(straight, false, options);
        const auto lost = img_handle(blue, false, options);
        check(!lost.seedFound && !lost.previewFound && !lost.pursuit.valid && lost.leftPoints == 0 && lost.rightPoints == 0,
              "valid-looking previous frame does not leak output into a lost-track frame");
        bool badSize = false;
        try { (void)img_handle(cv::Mat(240,320,CV_8UC3), false, options); } catch (const cv::Exception&) { badSize = true; }
        check(badSize, "wrong calibration resolution rejected");
        if (argc == 2) {
            const std::filesystem::path dir(argv[1]);
            if (!std::filesystem::create_directory(dir)) throw std::runtime_error("Fixture directory already exists");
            cv::imwrite((dir / "straight.png").string(), straight);
            cv::imwrite((dir / "gap.png").string(), gap);
            cv::VideoWriter writer((dir / "sequence.avi").string(), cv::VideoWriter::fourcc('M','J','P','G'),10,{640,480});
            if (!writer.isOpened()) throw std::runtime_error("Cannot create test video");
            for (int i=0; i<5; ++i) writer.write(straight);
            for (int i=0; i<5; ++i) writer.write(blue);
            for (int i=0; i<5; ++i) writer.write(straight);
        }
        std::cout << "All frontend tests passed. Synthetic images do not validate driving.\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
