[![Crates.io](https://meritbadge.herokuapp.com/linestring)](https://crates.io/crates/linestring)
[![Documentation](https://docs.rs/linestring/badge.svg)](https://docs.rs/linestring)
[![Workflow](https://github.com/eadf/linestring.rs/workflows/Rust/badge.svg)](https://github.com/eadf/linestring.rs/workflows/Rust/badge.svg)
[![Workflow](https://github.com/eadf/linestring.rs/workflows/Clippy/badge.svg)](https://github.com/eadf/linestring.rs/workflows/Clippy/badge.svg)
[![dependency status](https://deps.rs/crate/linestring/0.0.5/status.svg)](https://deps.rs/crate/linestring/0.0.5)

# Work in progress

Anything in this API ~~may~~ *will* change without notice.

This crate will contain data structures and methods that deals with lines in 2d and 3d space.

There will be 3d and 2d implementations of:
* Line, a finite two-point struct (no rays)
* LineString, a sequence of points, aka Polyline.
* [Ramer–Douglas-Peucker](https://en.wikipedia.org/wiki/Ramer–Douglas–Peucker_algorithm) and [Visvalingam-Whyatt](https://en.wikipedia.org/wiki/Visvalingam–Whyatt_algorithm) line simplification algorithms.
* Aabb [axis aligned bounding box](https://en.wikipedia.org/wiki/Minimum_bounding_box)

This will be implemented (feature gated) for [cgmath](https://crates.io/crates/cgmath), 
[nalgebra](https://crates.io/crates/nalgebra) and limited versions for [mint](https://crates.io/crates/nalgebra) and plain vector scalars (no transformations etc).
More implementations could be added if required.

If you want to use this library in your cgmath project you add this to your Cargo.toml:
```cargo
linestring = {version = "^0.0.5", features = ["impl-cgmath"]}
```
Same thing for the other supported 2d/3d libraries: 
```cargo
linestring = {version = "^0.0.5", features = ["impl-nalgebra"]}
```

```cargo
linestring = {version = "^0.0.5", features = ["impl-mint"]}
```

```cargo
linestring = {version = "^0.0.5", features = ["impl-vec"]}
```

Run the line simplification example with :
```fish
cargo run --example fltk_gui --features impl-cgmath
```

## Todo
- [ ] Improve on error handling
- [ ] Benchmark and optimize (add smallvec to simplify?)
- [ ] optimize Ramer–Douglas-Peucker 2d & 3d
- [ ] Visvalingam-Whyatt 2d & 3d
- [ ] Figure out why workflows refuse to start

