/*
Line and line segment library for 2d and 3d.

Copyright (C) 2021 eadf https://github.com/eadf

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.

Also add information on how to contact you by electronic and paper mail.

If the program does terminal interaction, make it output a short notice like
this when it starts in an interactive mode:

Linestring Copyright (C) 2021 eadf

This program comes with ABSOLUTELY NO WARRANTY; for details type `show w'.

This is free software, and you are welcome to redistribute it under certain
conditions; type `show c' for details.

The hypothetical commands `show w' and `show c' should show the appropriate
parts of the General Public License. Of course, your program's commands might
be different; for a GUI interface, you would use an "about box".

You should also get your employer (if you work as a programmer) or school,
if any, to sign a "copyright disclaimer" for the program, if necessary. For
more information on this, and how to apply and follow the GNU GPL, see <https://www.gnu.org/licenses/>.

The GNU General Public License does not permit incorporating your program
into proprietary programs. If your program is a subroutine library, you may
consider it more useful to permit linking proprietary applications with the
library. If this is what you want to do, use the GNU Lesser General Public
License instead of this License. But first, please read <https://www.gnu.org/
licenses /why-not-lgpl.html>.
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
    unused_variables
)]
#![cfg_attr(feature = "map_first_last", feature(map_first_last))]

//! This library contains data structures and methods that deals with lines in 2D and 3D space.
//!
//! There are 3D and 2D implementations of:
//! * LineString, a sequence of points, aka Polyline.
//! * Line, a finite two-point struct (no rays).
//! * [Ramer–Douglas-Peucker](https://en.wikipedia.org/wiki/Ramer–Douglas–Peucker_algorithm) and
//! * [Visvalingam-Whyatt](https://en.wikipedia.org/wiki/Visvalingam–Whyatt_algorithm) line simplification algorithms.
//! * Sampling of [boostvoronoi](https://github.com/eadf/boostvoronoi.rs) parabolic arc curves.
//! * Rudimentary functionality to save to .obj file
//!
//! There are 2D implementations of:
//! * LineString2 convex hull calculation (gift wrapping & Graham scan)
//! * Aabb [axis aligned bounding box](https://en.wikipedia.org/wiki/Minimum_bounding_box).
//! * Self intersection tests for line strings, or groups of lines O( n log n + i log n).
//! * Convex hull containment test (single threaded or multi-threaded with [ryon](https://crates.io/crates/rayon))
//! * Simple affine transformation (pan, zoom)
//!
//! The library is implemented for a couple of independent 2d/3d packages:nalgebra,cgmath,vecmath,mint and plain vectors.
//! Those implementations are feature gated so you only need to import the package you really use.
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