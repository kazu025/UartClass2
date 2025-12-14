#!/usr/bin/env bash
cd build-pico
make -j4 send
cd ..
cd build-picow
make -j4 recv
cd ..
