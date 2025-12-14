#!/usr/bin/env bash
rm -rf build-pico
mkdir build-pico && cd $_
cmake ..
make -j4 send
