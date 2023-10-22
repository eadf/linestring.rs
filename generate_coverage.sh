#!/bin/bash

# Ensure necessary tools are installed
command -v cargo >/dev/null 2>&1 || { echo >&2 "cargo is not installed. Aborting."; exit 1; }
command -v grcov >/dev/null 2>&1 || { echo >&2 "grcov is not installed. Aborting."; exit 1; }

# Set environment variables for Rust to use the LLVM coverage approach
export CARGO_INCREMENTAL=0
export RUSTFLAGS="-C instrument-coverage"
export LLVM_PROFILE_FILE='target/debug/coverage/%p-%m.profraw'

# Clean previous coverage data
cargo clean

# Build the project
cargo build

# Run tests
cargo test

# Capture the coverage data
grcov . -s . --binary-path ./target/debug/ -t html --branch --ignore-not-existing -o ./target/debug/coverage/

#list the index.html files
find ./target -iname "index.html"

# Open the coverage report in the default web browser
case $(uname) in
    "Linux") xdg-open ./target/debug/coverage/html/index.html ;;
    "Darwin") open ./target/debug/coverage/html/index.html ;;
    "CYGWIN"|"MINGW"|"MSYS") start ./target/debug/coverage/html/index.html ;;
    *) echo "Unable to open the coverage report automatically. Please open ./target/debug/coverage/html/index.html manually." ;;
esac
