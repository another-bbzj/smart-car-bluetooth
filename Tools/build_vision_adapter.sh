#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")/.."
mkdir -p build
if ! pkg-config --exists opencv4; then
    echo 'Missing OpenCV development files (libopencv-dev and pkg-config).' >&2
    exit 1
fi
read -r -a flags <<< "$(pkg-config --cflags --libs opencv4)"
g++ -std=c++17 -O2 -Wall -Wextra Tools/vision_adapter.cpp -o build/vision_adapter "${flags[@]}" -pthread
g++ -std=c++17 -O2 -Wall -Wextra -Werror Tests/test_vision_wire.cpp -o build/test_vision_wire
g++ -std=c++17 -O2 -Wall -Wextra -Werror Tests/test_vision_serial.cpp -o build/test_vision_serial
./build/test_vision_wire
./build/test_vision_serial
