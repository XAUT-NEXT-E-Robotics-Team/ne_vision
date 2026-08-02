#!/bin/bash

# Fix the empty paragraph error in Arrow C++ source code caused by Clang's strict -Werror,-Wdocumentation
FILE_PATH="/Users/ziyu/codes/ne_vision/build/_deps/rerun_sdk-build/arrow/src/arrow_cpp/cpp/src/arrow/util/ree_util.h"

if [ -f "$FILE_PATH" ]; then
    echo "Fixing ree_util.h documentation error..."
    # Replace the \par with standard text or add text after it to avoid empty paragraph
    sed -i '' 's/\\par You can write your loops like this instead:/You can write your loops like this instead:/g' "$FILE_PATH"
    echo "Done! You can run the build again."
else
    echo "Error: File $FILE_PATH not found. Are you sure the rerun_sdk build failed at this step?"
    exit 1
fi

# Fix Apple libtool version detection: newer Xcode CLT reports
# "Apple Inc. version cctools_ld-XXXX" instead of the old "cctools-X.X.X",
# which breaks BuildUtils.cmake's regex and aborts with a false
# "incompatible GNU libtool" error.
BUILDUTILS_CMAKE="/Users/ziyu/codes/ne_vision/build/_deps/rerun_sdk-build/arrow/src/arrow_cpp/cpp/cmake_modules/BuildUtils.cmake"

if [ -f "$BUILDUTILS_CMAKE" ]; then
    echo "Fixing BuildUtils.cmake libtool version regex..."
    sed -i '' 's/\.\*cctools-(\[0-9\.\]+)\.\*/.*cctools[-_][a-zA-Z0-9.]+.*/g' "$BUILDUTILS_CMAKE"
    echo "Done! You can run the build again."
else
    echo "Error: File $BUILDUTILS_CMAKE not found. Are you sure the rerun_sdk build failed at this step?"
    exit 1
fi

# Fix deprecated literal-operator syntax in Arrow's vendored date.h:
# `operator "" _d` (space before the suffix) is deprecated under newer
# Clang and becomes a hard error with -Werror,-Wdeprecated-literal-operator.
# Rewrite to the non-deprecated `operator""_d` form.
DATE_H="/Users/ziyu/codes/ne_vision/build/_deps/rerun_sdk-build/arrow/src/arrow_cpp/cpp/src/arrow/vendored/datetime/date.h"

if [ -f "$DATE_H" ]; then
    echo "Fixing date.h deprecated literal operator syntax..."
    sed -i '' -E 's/operator[[:space:]]*"" *(_[a-zA-Z_]+)/operator""\1/g' "$DATE_H"
    echo "Done! You can run the build again."
else
    echo "Error: File $DATE_H not found. Are you sure the rerun_sdk build failed at this step?"
    exit 1
fi
