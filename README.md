[![Crates.io](https://meritbadge.herokuapp.com/linestring)](https://crates.io/crates/linestring)
[![Documentation](https://docs.rs/linestring/badge.svg)](https://docs.rs/linestring)
[![Workflow](https://github.com/eadf/linestring.rs/workflows/Rust/badge.svg)](https://github.com/eadf/linestring.rs/workflows/Rust/badge.svg)
[![Workflow](https://github.com/eadf/linestring.rs/workflows/Clippy/badge.svg)](https://github.com/eadf/linestring.rs/workflows/Clippy/badge.svg)
[![dependency status](https://deps.rs/crate/linestring/0.0.7/status.svg)](https://deps.rs/crate/linestring/0.0.7)

# Work in progress

Anything in this API ~~may~~ *will* change without notice.

This crate will contain data structures and methods that deals with lines in 2d and 3d space.

There will be 3d and 2d implementations of:
- [x] LineString, a sequence of points, aka Polyline.
- [x] Self intersection tests in 2d
- [x] Line, a finite two-point struct (no rays)
- [x] [Ramer–Douglas-Peucker](https://en.wikipedia.org/wiki/Ramer–Douglas–Peucker_algorithm) and
- [x] [Visvalingam-Whyatt](https://en.wikipedia.org/wiki/Visvalingam–Whyatt_algorithm) line simplification algorithms.
- [x] Aabb [axis aligned bounding box](https://en.wikipedia.org/wiki/Minimum_bounding_box)
- [x] [boostvoronoi](https://github.com/eadf/boostvoronoi.rs) parabolic arc curves

This will be implemented (feature gated) for [cgmath](https://crates.io/crates/cgmath), 
[nalgebra](https://crates.io/crates/nalgebra), [vecmath](https://crates.io/crates/vecmath) 
and limited versions for [mint](https://crates.io/crates/mint) and plain vectors (no transformations etc).
More implementations could be added if required.

If you want to use this library in your cgmath project you add this to your Cargo.toml:
```cargo
linestring = {version = "^0.0.7", features = ["impl-cgmath"]}
```
Same thing for the other supported 2d/3d libraries: 
```cargo
linestring = {version = "^0.0.7", features = ["impl-nalgebra"]}
```

```cargo
linestring = {version = "^0.0.7", features = ["impl-mint"]}
```

```cargo
linestring = {version = "^0.0.7", features = ["impl-vecmath"]}
```
```cargo
linestring = {version = "^0.0.7", features = ["impl-vec"]}
```
The difference between ```impl-vecmath``` and ```impl-vec``` is that ```impl-vecmath``` implements transformations.

Run the line simplification example with :
```fish
cargo run --example fltk_gui --features impl-cgmath
```

## Todo
- [ ] Improve on error handling
- [ ] Benchmark and optimize (add smallvec to simplify rdp?)
- [ ] optimize Ramer–Douglas-Peucker 2d & 3d
- [ ] optimize Visvalingam-Whyatt 2d & 3d

