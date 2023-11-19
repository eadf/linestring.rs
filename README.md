[![crates.io](https://img.shields.io/crates/v/linestring.svg)](https://crates.io/crates/linestring)
[![Documentation](https://docs.rs/linestring/badge.svg)](https://docs.rs/linestring)
[![Workflow](https://github.com/eadf/linestring.rs/workflows/Rust/badge.svg)](https://github.com/eadf/linestring.rs/workflows/Rust/badge.svg)
[![dependency status](https://deps.rs/crate/linestring/0.12.1/status.svg)](https://deps.rs/crate/linestring/0.12.1)
![license](https://img.shields.io/crates/l/linestring)

# Linestring: poly-lines for generic vectors.

`linestring` is a Rust crate providing data structures and methods for working with poly-lines and segments in 2D space.
Utilizing the `vector-traits` crate, it abstracts over different vector implementations allowing for seamless
transitions between vector libraries like `glam` and `cgmath` (for now).

## Features

- **Generic Vector Support:** Seamlessly switch between different vector implementations.
- **Simple API**, most of the functionality is implemented for `Vec<GenericVector2>` and `Vec<GenericVector3>`
- **Core Data Structures:** LineString, Line, and axis-aligned bounding box (Aabb).
- **Line Simplification Algorithms:** [Ramer–Douglas–Peucker](https://en.wikipedia.org/wiki/Ramer–Douglas–Peucker_algorithm) and [Visvalingam-Whyatt](https://en.wikipedia.org/wiki/Visvalingam–Whyatt_algorithm).
- **Curve Sampling:** Sampling of [boostvoronoi](https://github.com/eadf/boostvoronoi.rs) parabolic arc curves.
- **Convex Hull Calculation:** Gift wrapping & Graham scan algorithms.
- **Intersection Tests:** Self-intersection tests for line strings, or groups of lines with O( n log n + i log n) complexity.
- **Containment Test:** Convex hull containment test.

## Usage

Include the following dependencies in your `Cargo.toml` file, picking the vector implementation you need:
```toml
vector-traits = {version="0.3.2", features= ["glam","cgmath"]} # pick cgmath or glam, whatever you need
linestring = {version="0.12"}
```
## Example

```rust
// the vector type can just as well be glam::Dvec2, cgmath::Vector2<f32> or cgmath::Vector2<f64>,
let some_points: Vec<glam::Vec2> = vec![
    vec2(77f32, 613.),
    vec2(689., 650.),
    vec2(710., 467.),
    vec2(220., 200.),
];
let convex_hull:Vec<glam::Vec2> = some_points.convex_hull()?;
for p in some_points {
    assert!(convex_hull.contains_point_inclusive(p));
}
```

### Minimum Supported Rust Version (MSRV)

The minimum supported version of Rust for `linestring` is `1.66`.

## Contributing

We welcome contributions from the community.
Feel free to submit pull requests or report issues on our GitHub repository.
Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, 
as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.

## License

Licensed under either of

* Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE)
  or http://www.apache.org/licenses/LICENSE-2.0)
* MIT license ([LICENSE-MIT](LICENSE-MIT)
  or http://opensource.org/licenses/MIT)

at your option.
