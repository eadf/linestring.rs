[![Crates.io](https://meritbadge.herokuapp.com/linestring)](https://crates.io/crates/linestring)
[![Documentation](https://docs.rs/linestring/badge.svg)](https://docs.rs/linestring)
[![Workflow](https://github.com/eadf/linestring.rs/workflows/Rust/badge.svg)](https://github.com/eadf/linestring.rs/workflows/Rust/badge.svg)
[![Workflow](https://github.com/eadf/linestring.rs/workflows/Clippy/badge.svg)](https://github.com/eadf/linestring.rs/workflows/Clippy/badge.svg)
[![dependency status](https://deps.rs/crate/linestring/0.0.8/status.svg)](https://deps.rs/crate/linestring/0.0.8)

# Line library for Rust

This crate contains data structures and methods that deals with lines in 2D and 3D space.

There are 3D and 2D implementations of:
- [x] LineString, a sequence of points, aka Polyline.
- [x] Self intersection tests for lines in 2d, O( n log n + k).
- [x] Line, a finite two-point struct (no rays).
- [x] [Ramer–Douglas-Peucker](https://en.wikipedia.org/wiki/Ramer–Douglas–Peucker_algorithm) and
- [x] [Visvalingam-Whyatt](https://en.wikipedia.org/wiki/Visvalingam–Whyatt_algorithm) line simplification algorithms.
- [x] Aabb [axis aligned bounding box](https://en.wikipedia.org/wiki/Minimum_bounding_box).
- [x] sampling of [boostvoronoi](https://github.com/eadf/boostvoronoi.rs) parabolic arc curves.
- [ ] 2D Convex hull and containment tests

This is implemented for [cgmath](https://crates.io/crates/cgmath), 
[nalgebra](https://crates.io/crates/nalgebra), [vecmath](https://crates.io/crates/vecmath) 
and limited versions for [mint](https://crates.io/crates/mint) and plain vectors (no transformations etc).
\
These implementations are feature gated, so you only have to include the 3d package you need in your code.
More implementations could be added if required.

If you want to use this library in your cgmath project you add this to your Cargo.toml:
```cargo
linestring = {version = "^0.0.8", features = ["impl-cgmath"]}
```
Same thing for the other supported 2d/3d libraries: 
```cargo
linestring = {version = "^0.0.8", features = ["impl-nalgebra"]}
```

```cargo
linestring = {version = "^0.0.8", features = ["impl-mint"]}
```

```cargo
linestring = {version = "^0.0.8", features = ["impl-vecmath"]}
```
```cargo
linestring = {version = "^0.0.8", features = ["impl-vec"]}
```
The difference between ```impl-vecmath``` and ```impl-vec``` is that the ```impl-vecmath``` feature supports 
affine transformations.

##Demo
Run the line simplification example with :
```fish
cargo run --example fltk_gui --features impl-cgmath
```

## Todo
- [ ] Improve on error handling
- [ ] Benchmark and optimize (add smallvec to simplify rdp?)
- [ ] optimize Ramer–Douglas-Peucker
- [ ] optimize Visvalingam-Whyatt
- [ ] figure out how to deal with rustdoc (the feature gates disables it). 

