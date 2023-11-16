// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2021,2023 lacklustr@protonmail.com https://github.com/eadf

// This file is part of the linestring crate.

/*
Copyright (c) 2021,2023 lacklustr@protonmail.com https://github.com/eadf

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

or

Copyright 2021,2023 lacklustr@protonmail.com https://github.com/eadf

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#![deny(
    rust_2018_compatibility,
    rust_2018_idioms,
    nonstandard_style,
    unused,
    future_incompatible,
    non_camel_case_types,
    unused_parens,
    non_upper_case_globals,
    unused_qualifications,
    unused_results,
    unused_imports,
    unused_variables,
    bare_trait_objects,
    ellipsis_inclusive_range_patterns,
    elided_lifetimes_in_paths
)]
#![warn(clippy::explicit_into_iter_loop)]

//! This library contains data structures and methods that deals with lines in 2D and 3D space.
//!
//! There are 2D implementations of:
//! * LineString, a sequence of points, aka Polyline.
//! * Line, a finite two-point struct (no rays).
//! * [Ramer–Douglas-Peucker](https://en.wikipedia.org/wiki/Ramer–Douglas–Peucker_algorithm) and
//! * [Visvalingam-Whyatt](https://en.wikipedia.org/wiki/Visvalingam–Whyatt_algorithm) line simplification algorithms.
//! * Sampling of [boostvoronoi](https://github.com/eadf/boostvoronoi.rs) parabolic arc curves.
//! * LineString2 convex hull calculation (gift wrapping & Graham scan)
//! * Aabb [axis aligned bounding box](https://en.wikipedia.org/wiki/Minimum_bounding_box).
//! * Self intersection tests for line strings, or groups of lines O( n log n + i log n).
//! * Convex hull containment test (single threaded or multi-threaded with [ryon](https://crates.io/crates/rayon))
//! * Simple affine transformation (pan, zoom)
//!
use std::fmt::Debug;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum LinestringError {
    #[error("Your line-strings are self-intersecting.")]
    SelfIntersectingData(String),

    #[error("The input data is not 2D")]
    InputNotPLane(String),

    #[error("Invalid data")]
    InvalidData(String),

    #[error("Unknown error")]
    InternalError(String),

    #[error("Aabb error")]
    AabbError(String),

    #[error("Aabb error")]
    TransformError(String),

    #[error(transparent)]
    IoError(#[from] std::io::Error),
}

pub mod linestring_2d;
pub mod linestring_3d;
pub mod shape;

pub mod prelude {
    pub use crate::{
        linestring_2d::LineString2, linestring_3d::LineString3, shape::divide_into_shapes,
    };
}
