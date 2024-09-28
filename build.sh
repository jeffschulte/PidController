#!/bin/bash

rm -rf build
mkdir build
cd build
cmake ..
make
export LD_LIBRARY_PATH=/home/jeff/Desktop/PidController/build/src:$LD_LIBRARY_PATH
cd ..
