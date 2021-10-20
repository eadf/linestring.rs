[![crates.io](https://img.shields.io/crates/v/linestring.svg)](https://crates.io/crates/linestring)
[![Documentation](https://docs.rs/linestring/badge.svg)](https://docs.rs/linestring)
[![Workflow](https://github.com/eadf/linestring.rs/workflows/Rust/badge.svg)](https://github.com/eadf/linestring.rs/workflows/Rust/badge.svg)
[![Workflow](https://github.com/eadf/linestring.rs/workflows/Clippy/badge.svg)](https://github.com/eadf/linestring.rs/workflows/Clippy/badge.svg)
[![dependency status](https://deps.rs/crate/linestring/0.7.1/status.svg)](https://deps.rs/crate/linestring/0.7.1)
![license](https://img.shields.io/crates/l/linestring)

# Lackadaisical 2d and 3d polylines for cgmath

Data structures and methods for polylines/segments in 2D and 3D space.

There are 3D and 2D implementations of:
* LineString, a sequence of points, aka Polyline.
* Line, a finite two-point struct (no rays).
* [Ramer–Douglas-Peucker](https://en.wikipedia.org/wiki/Ramer–Douglas–Peucker_algorithm) and
* [Visvalingam-Whyatt](https://en.wikipedia.org/wiki/Visvalingam–Whyatt_algorithm) line simplification algorithms.
* Sampling of [boostvoronoi](https://github.com/eadf/boostvoronoi.rs) parabolic arc curves.
* Rudimentary functionality to save to .obj file

There are 2D implementations of:
* LineString2 convex hull calculation (gift wrapping & Graham scan)
* Aabb [axis aligned bounding box](https://en.wikipedia.org/wiki/Minimum_bounding_box).
* Self intersection tests for line strings, or groups of lines O( n log n + i log n).
* Convex hull containment test
* Simple affine transformation (pan, zoom)

If you want to use this library in your cgmath project you add this to your Cargo.toml:
```cargo
linestring = {version="0.7"}
```

## Rust toolchains

This crate uses `#![feature(map_first_last)]` if compiled by +nightly. This is functionality is only emulated when run on +stable.

## Demo GUI
Run the line simplification example with :
```fish
cargo run --example fltk_gui
```

## Todo
- [ ] Improve on error handling
- [ ] Benchmark and optimize (add smallvec to simplify rdp?)
- [ ] optimize Ramer–Douglas-Peucker
- [ ] optimize Visvalingam-Whyatt
- [ ] Stable overlapping co-linear line detection
- [ ] Add subdivide
