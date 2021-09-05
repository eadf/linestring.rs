#!/bin/sh

# perform each test suite with a limited feature set, then once with every feature enabled
cargo test --no-default-features && \
cargo test --no-default-features --features rayon

