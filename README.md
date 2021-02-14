[![Crates.io](https://meritbadge.herokuapp.com/linestring)](https://crates.io/crates/linestring)
[![Documentation](https://docs.rs/linestring/badge.svg)](https://docs.rs/linestring)
[![Workflow](https://github.com/eadf/linestring.rs/workflows/Rust/badge.svg)](https://github.com/eadf/linestring.rs/workflows/Rust/badge.svg)
[![Workflow](https://github.com/eadf/linestring.rs/workflows/Clippy/badge.svg)](https://github.com/eadf/linestring.rs/workflows/Clippy/badge.svg)
[![dependency status](https://deps.rs/crate/linestring/0.0.2/status.svg)](https://deps.rs/crate/linestring/0.0.2)

# Work in progress

Anything in this API ~~may~~ *will* change without notice.

This crate will contain data structures and methods that deals with lines in 2d and 3d geometry.

There will be 3d and 2d implementations of:
* Line, a finite two-point struct
* LineString, a sequence of points
* [Ramer–Douglas-Peucker](https://en.wikipedia.org/wiki/Ramer–Douglas–Peucker_algorithm) and [Visvalingam-Whyatt](https://en.wikipedia.org/wiki/Visvalingam–Whyatt_algorithm) line simplification algorithms.
* Aabb [axis aligned bounding box](https://en.wikipedia.org/wiki/Minimum_bounding_box)
* Maybe [2d line intersection](https://github.com/eadf/intersect2d.rs)

This will be implemented (feature gated) for [cgmath](https://crates.io/crates/cgmath), 
[nalgebra](https://crates.io/crates/nalgebra) and a limited version for [mint](https://crates.io/crates/nalgebra) (no transformations etc).
More implementations could be added if required.

Cargo.toml usage:
```cargo
linestring = {version = "^0.0.2", features = ["impl-cgmath"]}
```

```cargo
linestring = {version = "^0.0.2", features = ["impl-nalgebra"]}
```

```cargo
linestring = {version = "^0.0.2", features = ["impl-mint"]}
```

