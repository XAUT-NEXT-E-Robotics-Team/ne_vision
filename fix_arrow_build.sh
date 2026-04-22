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
