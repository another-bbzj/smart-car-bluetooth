// Standalone transport entry point. Vision/ is an unchanged source snapshot
// of the supplied vision.rar; its algorithms, configuration and main stay intact.
#include "../Vision/img_handle.hpp"
#include "vision_serial.hpp"
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <thread>

static volatile std::sig_atomic_t stopping = 0;
static void stop(int) { stopping = 1; }

int main(int argc, char** argv)
{
    using Clock = std::chrono::steady_clock;
    try {
        std::string source, input, serialPath, configPath, wirePath;
        bool calibrated = false;
        unsigned maxFrames = 0;
        for (int i = 1; i < argc; ++i) {
            const std::string key = argv[i];
            auto value = [&]() -> std::string {
                if (++i >= argc) throw std::invalid_argument("Missing value: " + key);
                return argv[i];
            };
            if (key == "--help") {
                std::cout << "vision_adapter (--camera INDEX | --video FILE) [--config FILE]\n"
                          << "  --serial DEVICE --calibrated  live 115200 8N1; camera only\n"
                          << "  --wire-output NEW_FILE        offline binary protocol capture\n"
                          << "  --max-frames N                0 = unlimited\n"
                          << "Run from the Vision directory so unchanged calibration paths resolve.\n"
                          << "No motor/start commands. TYPE 3 keeps original right-positive curvature.\n";
                return 0;
            } else if (key == "--camera" || key == "--video") {
                if (!source.empty()) throw std::invalid_argument("Choose one input source");
                source = key; input = value();
            } else if (key == "--serial") serialPath = value();
            else if (key == "--config") configPath = value();
            else if (key == "--wire-output") wirePath = value();
            else if (key == "--calibrated") calibrated = true;
            else if (key == "--max-frames") {
                const auto text = value();
                std::size_t used = 0;
                const int n = std::stoi(text, &used);
                if (used != text.size() || n < 0) throw std::invalid_argument("Invalid frame limit");
                maxFrames = static_cast<unsigned>(n);
            } else throw std::invalid_argument("Unknown option: " + key);
        }
        if (source.empty()) throw std::invalid_argument("An input source is required");
        if (!serialPath.empty() && (source != "--camera" || !calibrated))
            throw std::invalid_argument("Serial needs a live camera and explicit --calibrated confirmation");
        if (!wirePath.empty() && std::filesystem::exists(wirePath))
            throw std::invalid_argument("Wire output already exists");
        ProcessOptions options;
        options.mode = TrackMode::Hsv;
        if (!configPath.empty()) options.frontend = track::loadConfig(configPath);
        cv::VideoCapture capture;
        if (source == "--camera") {
            std::size_t used = 0;
            const int index = std::stoi(input, &used);
            if (index < 0 || used != input.size()) throw std::invalid_argument("Invalid camera index");
            if (!capture.open(index, cv::CAP_V4L2)) throw std::runtime_error("Cannot open camera");
            capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
            capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
            capture.set(cv::CAP_PROP_BUFFERSIZE, 1); // Backend may ignore; verify capture latency on hardware.
        } else if (!capture.open(input)) throw std::runtime_error("Cannot open video");
        std::unique_ptr<visionlink::Serial> serial;
        if (!serialPath.empty()) serial = std::make_unique<visionlink::Serial>(serialPath);
        std::ofstream wire;
        if (!wirePath.empty()) {
            wire.open(wirePath, std::ios::binary);
            if (!wire) throw std::runtime_error("Cannot create wire output");
        }
        std::random_device random;
        const std::uint32_t session = static_cast<std::uint32_t>(random()) | 1U;
        const auto hello = visionlink::hello(session);
        if (wire.is_open()) wire.write(reinterpret_cast<const char*>(hello.data()), hello.size());
        if (serial) {
            const auto deadline = Clock::now() + std::chrono::milliseconds(200);
            while (!serial->send(hello)) {
                if (Clock::now() >= deadline) throw std::runtime_error("Cannot send HELLO");
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
        std::signal(SIGINT, stop); std::signal(SIGTERM, stop);
        resetVisionState();
        std::uint16_t sequence = 0;
        unsigned frames = 0, valid = 0, dropped = 0;
        while (!stopping && (!maxFrames || frames < maxFrames)) {
            // Include capture wait and processing, rather than reporting algorithm time alone.
            const auto acquired = Clock::now();
            cv::Mat image;
            if (!capture.read(image) || image.empty()) {
                if (source == "--camera" || frames == 0) throw std::runtime_error("Input read failed");
                break;
            }
            auto result = img_handle(image, false, options);
            const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - acquired).count();
            const unsigned age = static_cast<unsigned>(std::min<std::int64_t>(elapsed, 65535));
            const bool good = result.seedFound && result.previewFound && result.pursuit.valid;
            const auto next = static_cast<std::uint16_t>(sequence + 1);
            const auto bytes = visionlink::track(session, next, good, result.pursuit.curvature, age);
            if (wire.is_open()) {
                wire.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
                if (!wire) throw std::runtime_error("Wire output write failed");
            }
            if (!serial || serial->send(bytes)) sequence = next;
            else ++dropped;
            valid += bytes[15] != 0;
            ++frames;
        }
        if (serial) {
            // Best effort invalidation; absence of further frames also trips the firmware watchdog.
            (void)serial->send(visionlink::track(session, static_cast<std::uint16_t>(sequence + 1), false, 0, 0));
        }
        if (wire.is_open()) { wire.flush(); if (!wire) throw std::runtime_error("Wire output flush failed"); }
        std::cout << "frames=" << frames << " valid=" << valid << " serial_dropped=" << dropped
                  << " session=" << session << '\n';
        return frames ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "Adapter stopped: " << error.what() << '\n';
        return 1; // F407 stops on stale vision, never auto-rearms.
    }
}
