#!/bin/sh

# perform each test suite with a limited feature set, then once with every feature enabled
cargo test --no-default-features --features cgmath && \
cargo test --no-default-features --features cgmath,rayon && \
cargo test --no-default-features --features vector && \
cargo test --no-default-features --features vector,rayon && \
cargo test --no-default-features --features mint && \
cargo test --no-default-features --features mint,rayon && \
cargo test --no-default-features --features vecmath && \
cargo test --no-default-features --features vecmath,rayon && \
cargo test --no-default-features --features nalgebra && \
cargo test --no-default-features --features nalgebra,rayon && \
cargo test --no-default-features --features cgmath,vecmath && \
cargo test --no-default-features --features cgmath,vecmath,rayon && \
cargo test
