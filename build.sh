#!/usr/bin/env bash
rm -rf release
mkdir release
cd release
cmake -D CMAKE_BUILD_TYPE=RELEASE -DCMAKE_PREFIX_PATH=$HOME/Qt/5.7/clang_64/lib/cmake/ ..
make -j4