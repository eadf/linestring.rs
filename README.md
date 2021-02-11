[![Crates.io](https://meritbadge.herokuapp.com/linestring)](https://crates.io/crates/linestring)
[![Documentation](https://docs.rs/linestring/badge.svg)](https://docs.rs/linestring)
[![Workflow](https://github.com/eadf/linestring.rs/workflows/Rust/badge.svg)](https://github.com/eadf/linestring.rs/workflows/Rust/badge.svg)
[![Workflow](https://github.com/eadf/linestring.rs/workflows/Clippy/badge.svg)](https://github.com/eadf/linestring.rs/workflows/Clippy/badge.svg)
[![dependency status](https://deps.rs/crate/linestring/0.0.2/status.svg)](https://deps.rs/crate/linestring/0.0.2)

#Work in progress
Anything in this API may change without notice.

This is intended to be 3d and 2d implementations of:
* Line
* LineString
* Aabb [axis aligned bounding box](https://en.wikipedia.org/wiki/Minimum_bounding_box)
* [Ramer–Douglas-Peucker](https://en.wikipedia.org/wiki/Ramer–Douglas–Peucker_algorithm) and [Visvalingam-Whyatt](https://en.wikipedia.org/wiki/Visvalingam–Whyatt_algorithm) line simplification algorithms.

This will be implemented (feature gated) for cgmath, nalgebra and mint (at least).

Cargo.toml usage:
```cargo
linestring = {version = "^0.0.2", features = ["impl-cgmath"]}
```

```cargo
linestring = {version = "^0.0.2", features = ["impl-mint"]}
```

```cargo
linestring = {version = "^0.0.2", features = ["impl-nalgebra"]}
```


