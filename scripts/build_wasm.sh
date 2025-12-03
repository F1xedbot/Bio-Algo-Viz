#!/bin/bash

OUTPUT_DIR="docs"

echo "Building WASM..."

GOOS=js GOARCH=wasm go build -o $OUTPUT_DIR/main.wasm ./cmd/app

# Check if file exists in web/, if not, download it
if [ ! -f "$OUTPUT_DIR/wasm_exec.js" ]; then
    echo "wasm_exec.js not found, downloading from Go repo..."
    curl -s -o $OUTPUT_DIR/wasm_exec.js https://raw.githubusercontent.com/golang/go/master/misc/wasm/wasm_exec.js
fi

echo "Build complete. Files are in '$OUTPUT_DIR/'"