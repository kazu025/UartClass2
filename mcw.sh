#!/usr/bin/env bash
rm -rf build-picow
mkdir build-picow && cd $_
cmake .. -DPICO_BOARD=pico_w
make -j4 recv
