[![crates.io](https://img.shields.io/crates/v/linestring.svg)](https://crates.io/crates/linestring)
[![Documentation](https://docs.rs/linestring/badge.svg)](https://docs.rs/linestring)
[![Workflow](https://github.com/eadf/linestring.rs/workflows/Rust/badge.svg)](https://github.com/eadf/linestring.rs/workflows/Rust/badge.svg)
[![dependency status](https://deps.rs/crate/linestring/0.10.2/status.svg)](https://deps.rs/crate/linestring/0.10.2)
![license](https://img.shields.io/crates/l/linestring)
[![](https://img.shields.io/static/v1?label=Sponsor&message=%E2%9D%A4&logo=GitHub&color=%23fe8e86)](https://github.com/sponsors/eadf)

# Linestring: 2D poly-lines for generic vectors.

`linestring` is a Rust crate providing data structures and methods for working with poly-lines and segments in 2D space.
Utilizing the `vector-traits` crate, it abstracts over different vector implementations allowing for seamless
transitions between vector libraries like `glam` and `cgmath` (for now).

## Features

- **Generic Vector Support:** Seamlessly switch between different vector implementations.
- **Core Data Structures:** LineString, Line, and axis-aligned bounding box (Aabb).
- **Line Simplification Algorithms:** [Ramer–Douglas–Peucker](https://en.wikipedia.org/wiki/Ramer–Douglas–Peucker_algorithm) and [Visvalingam-Whyatt](https://en.wikipedia.org/wiki/Visvalingam–Whyatt_algorithm).
- **Curve Sampling:** Sampling of [boostvoronoi](https://github.com/eadf/boostvoronoi.rs) parabolic arc curves.
- **Convex Hull Calculation:** Gift wrapping & Graham scan algorithms.
- **Intersection Tests:** Self-intersection tests for line strings, or groups of lines with O( n log n + i log n) complexity.
- **Containment Test:** Convex hull containment test.
- **Affine Transformations:** Simple affine transformations including pan and zoom.

## Usage

Include the following dependencies in your `Cargo.toml` file, picking the vector implementation you need:
```toml
vector-traits = {version="0.2", features= ["glam","cgmath"]} # pick cgmath or glam, whatever you need
linestring = {version="0.10"}
```

## Demo GUI
Run the line simplification example with :
```shell
cargo run --example fltk_gui
```

## Todo
- [ ] Improve on error handling
- [x] optimize Ramer–Douglas-Peucker
- [x] optimize Visvalingam-Whyatt
- [ ] Stable overlapping co-linear line detection
- [x] Use Point-by-value instead of `&Point`

### Minimum Supported Rust Version (MSRV)

The minimum supported version of Rust for `linestring` is `1.66`.

## Contributing

We welcome contributions from the community.
Feel free to submit pull requests or report issues on our GitHub repository.

## License

Licensed under either of

* Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE)
  or http://www.apache.org/licenses/LICENSE-2.0)
* MIT license ([LICENSE-MIT](LICENSE-MIT)
  or http://opensource.org/licenses/MIT)

at your option.
