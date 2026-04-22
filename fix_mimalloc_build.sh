#!/bin/bash

# Fix mimalloc CMake compatibility issue with newer CMake versions
MIMALLOC_CMAKE="/Users/ziyu/codes/ne_vision/build/_deps/rerun_sdk-build/arrow/src/arrow_cpp-build/mimalloc_ep-prefix/src/mimalloc_ep/CMakeLists.txt"

if [ -f "$MIMALLOC_CMAKE" ]; then
    echo "Fixing mimalloc CMakeLists.txt version requirement..."
    # Replace "cmake_minimum_required(VERSION 2.8.12)" with "cmake_minimum_required(VERSION 3.5)"
    sed -i '' 's/cmake_minimum_required(VERSION [0-9.]*)/cmake_minimum_required(VERSION 3.5)/g' "$MIMALLOC_CMAKE"
    echo "Done! You can run the build again."
else
    echo "Error: File $MIMALLOC_CMAKE not found."
    exit 1
fi
