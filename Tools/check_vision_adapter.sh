#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")/.."
bash Tools/build_vision_adapter.sh
# Each run gets a fresh output directory; original sources and old fixtures stay intact.
fixture=$(mktemp -d "$PWD/build/vision-replay.XXXXXX")
(cd Vision && bash build.sh test_frontend && ./test_frontend "$fixture/images" && \
    ../build/vision_adapter --video "$fixture/images/sequence.avi" --wire-output "$fixture/wire.bin")
cc -std=c99 -Wall -Wextra -Werror -I CODE Tests/test_vision_replay.c \
    CODE/Car_Core.c CODE/Car_PID.c CODE/Car_Reference.c CODE/Car_Vision.c \
    -lm -o build/test_vision_replay
./build/test_vision_replay "$fixture/wire.bin"
