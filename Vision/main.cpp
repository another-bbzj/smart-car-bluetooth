#include <opencv2/opencv.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include "img_handle.hpp"

namespace fs = std::filesystem;

struct Arguments {
    std::string source, path, output, configPath;
    TrackMode mode = TrackMode::Hsv;
    bool tcp = false, show = false;
    int maxFrames = 0, saveEvery = 1;
};

void usage() {
    std::cout << "Usage: ./vision (--image FILE | --video FILE | --camera INDEX) [options]\n"
        "  --mode hsv|gray       default hsv; gray uses the assessment pipeline\n"
        "  --config FILE         HSV thresholds (otherwise built-in defaults)\n"
        "  --output NEW_DIR      CSV and diagnostic PNGs; directory must not exist\n"
        "  --save-every N        save every Nth frame, default 1 (CSV keeps all)\n"
        "  --max-frames N        stop after N frames, 0 means unlimited\n"
        "  --show               local windows; ESC/q quits, SPACE pauses replay\n"
        "  --tcp                enable original TCP viewer on port 9000\n"
        "No actuator commands are transmitted. Inputs must be 640x480.\n";
}

int integer(const std::string& text, const char* option, int minimum) {
    std::size_t used = 0;
    int n = std::stoi(text, &used);
    if (used != text.size() || n < minimum) throw std::invalid_argument(std::string("Invalid ") + option);
    return n;
}

Arguments parse(int argc, char** argv) {
    Arguments a;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        auto value = [&]() -> std::string {
            if (++i >= argc) throw std::invalid_argument("Missing value: " + key);
            return argv[i];
        };
        if (key == "--image" || key == "--video" || key == "--camera") {
            if (!a.source.empty()) throw std::invalid_argument("Choose exactly one input source");
            a.source = key; a.path = value();
        } else if (key == "--mode") {
            const auto v = value();
            if (v != "hsv" && v != "gray") throw std::invalid_argument("Mode must be hsv or gray");
            a.mode = v == "hsv" ? TrackMode::Hsv : TrackMode::Gray;
        } else if (key == "--config") a.configPath = value();
        else if (key == "--output") a.output = value();
        else if (key == "--max-frames") a.maxFrames = integer(value(), "--max-frames", 0);
        else if (key == "--save-every") a.saveEvery = integer(value(), "--save-every", 1);
        else if (key == "--tcp") a.tcp = true;
        else if (key == "--show") a.show = true;
        else throw std::invalid_argument("Unknown argument: " + key);
    }
    if (a.source.empty()) throw std::invalid_argument("An input source is required; use --help");
    return a;
}

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--help") { usage(); return 0; }
        const auto args = parse(argc, argv);
        ProcessOptions options;
        options.mode = args.mode;
        if (!args.configPath.empty()) options.frontend = track::loadConfig(args.configPath);
        if (args.mode == TrackMode::Hsv) {
            std::cerr << "HSV thresholds are provisional; validate them on venue recordings.\n";
        }
        cv::VideoCapture capture;
        cv::Mat image;
        if (args.source == "--image") {
            image = cv::imread(args.path, cv::IMREAD_COLOR);
            if (image.empty()) throw std::runtime_error("Cannot read image: " + args.path);
        } else if (args.source == "--video") {
            if (!capture.open(args.path)) throw std::runtime_error("Cannot open video: " + args.path);
        } else {
            const int index = integer(args.path, "--camera", 0);
            if (!capture.open(index, cv::CAP_V4L2)) throw std::runtime_error("Cannot open camera");
            capture.set(cv::CAP_PROP_FRAME_WIDTH, config::kInputWidth);
            capture.set(cv::CAP_PROP_FRAME_HEIGHT, config::kInputHeight);
        }
        if (args.tcp) remote.enable();
        std::ofstream csv;
        if (!args.output.empty()) {
            if (!fs::create_directory(args.output)) throw std::runtime_error("Output directory already exists");
            csv.open(fs::path(args.output) / "frames.csv");
            if (!csv) throw std::runtime_error("Cannot create frames.csv");
            csv << "frame,source_ms,processing_ms,mode,seed_found,seed_y,blue_support,left_points,right_points,cross,ring,side,preview_found,pp_valid,lateral_m,curvature,steering_deg\n";
        }
        resetVisionState();
        int frames = 0, valid = 0;
        using Clock = std::chrono::steady_clock;
        const auto start = Clock::now();
        double processTotalMs = 0;
        while (args.maxFrames == 0 || frames < args.maxFrames) {
            cv::Mat current;
            double sourceMs = 0;
            if (args.source == "--image") {
                if (frames > 0) break;
                current = image;
            } else {
                if (!capture.read(current) || current.empty()) {
                    if (args.source == "--camera" || frames == 0) throw std::runtime_error("Input read failed");
                    break; // end of a decoded video
                }
                sourceMs = args.source == "--video" ? capture.get(cv::CAP_PROP_POS_MSEC) :
                    std::chrono::duration<double, std::milli>(Clock::now() - start).count();
            }
            const auto t0 = Clock::now();
            const auto result = img_handle(current, false, options);
            const double ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
            processTotalMs += ms;
            valid += result.pursuit.valid;
            remote.imshow("Camera", current);
            if (csv.is_open()) {
                csv << frames << ',' << sourceMs << ',' << ms << ','
                    << (args.mode == TrackMode::Hsv ? "hsv" : "gray") << ','
                    << result.seedFound << ',' << result.seedY << ',' << result.blueSupport << ','
                    << result.leftPoints << ',' << result.rightPoints << ','
                    << static_cast<int>(result.cross) << ',' << bird::ringStateName(result.ring) << ','
                    << result.selectedSide << ',' << result.previewFound << ',' << result.pursuit.valid << ','
                    << result.pursuit.lateralError << ',' << result.pursuit.curvature << ','
                    << result.pursuit.steeringDegrees << '\n';
                if (!csv) throw std::runtime_error("CSV write failed");
                if (frames % args.saveEvery == 0) {
                    const std::string prefix = cv::format("%06d_", frames);
                    auto save = [&](const char* suffix, const cv::Mat& picture) {
                        if (!picture.empty() && !cv::imwrite((fs::path(args.output) / (prefix + suffix + ".png")).string(), picture))
                            throw std::runtime_error("Image write failed");
                    };
                    save("input", current); save("mask", result.mask); save("blue", result.blueMask);
                    save("edges", result.overlay); save("bird", result.birdImage);
                }
            }
            ++frames;
            if (args.show) {
                cv::imshow("Camera", current); cv::imshow("edges", result.overlay); cv::imshow("bird", result.birdImage);
                if (!result.mask.empty()) cv::imshow("mask", result.mask);
                if (!result.blueMask.empty()) cv::imshow("blue", result.blueMask);
                int delay = args.source == "--image" ? 0 : 1;
                if (args.source == "--video") {
                    const double fps = capture.get(cv::CAP_PROP_FPS);
                    if (std::isfinite(fps) && fps > 0) delay = std::clamp(static_cast<int>(1000.0 / fps - ms), 1, 1000);
                }
                int key = cv::waitKey(delay) & 0xff;
                if (key == ' ' && args.source == "--video") key = cv::waitKey(0) & 0xff;
                if (key == 27 || key == 'q') break;
            }
        }
        if (csv.is_open()) { csv.flush(); if (!csv) throw std::runtime_error("CSV flush failed"); }
        std::cout << "frames=" << frames << " pp_valid_frames=" << valid
                  << " mean_processing_ms=" << (frames ? processTotalMs / frames : 0)
                  << " (not a real-vehicle success rate)\n";
        return frames ? 0 : 1;
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << '\n';
        return 1;
    }
}
