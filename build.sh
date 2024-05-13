#!/bin/bash
if [ ! -d "build" ]; then
    mkdir build
fi
cd build
../configure --target-list=loongarch32-softmmu --disable-werror --enable-debug
make -j
