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

#![deny(non_camel_case_types)]
#![deny(unused_parens)]
#![deny(non_upper_case_globals)]
#![deny(unused_qualifications)]
#![deny(unused_results)]
#![deny(unused_imports)]

//! This is a library that deals with lines in 2D and 3D space.
//! The library is implemented for a couple of independent 2d/3d packages:nalgebra,cgmath,vecmath and mint.
//! Those implementations are feature gated so you only need to import the package you really use.
use thiserror::Error;

#[derive(Error, Debug)]
pub enum LinestringError {
    #[error("Your line-strings are self-intersecting.")]
    SelfIntersectingData { txt: String },

    #[error("The input data is not 2D")]
    InputNotPLane { txt: String },

    #[error("Invalid data")]
    InvalidData { txt: String },

    #[error("Unkown error")]
    UnknownError { txt: String },

    #[error("Aabb error")]
    AabbError { txt: String },

    #[error("Aabb error")]
    TransformError { txt: String },
}

/// This module is optional. To use if you must include the feature ```impl-nalgebra``` in your ```Cargo.toml``` file.
///
/// By default all features are enabled (this is a work-around to enable the automatic rust doc).
/// If you want to only enable the ```impl-nalgebra``` feature you must first disable the default features with ```default-features=false```.
///
/// Cargo.toml example: ```linestring = {version="<current version>",default-features=false,features=["impl-nalgebra"]}```
#[cfg(feature = "impl-nalgebra")]
pub mod nalgebra_2d;
/// This module is optional. To use if you must include the feature ```impl-nalgebra``` in your ```Cargo.toml``` file.
///
/// By default all features are enabled (this is a work-around to enable the automatic rust doc).
/// If you want to only enable the ```impl-nalgebra``` feature you must first disable the default features with ```default-features=false```.
///
/// Cargo.toml example: ```linestring = {version="<current version>",default-features=false,features=["impl-nalgebra"]}```
#[cfg(feature = "impl-nalgebra")]
pub mod nalgebra_3d;

/// This module is optional. To use if you must include the feature ```impl-cgmath``` in your ```Cargo.toml``` file.
///
/// By default all features are enabled (this is a work-around to enable the automatic rust doc).
/// If you want to only enable the ```impl-cgmath``` feature you must first disable the default features with ```default-features=false```.
///
/// Cargo.toml example: ```linestring = {version="<current version>",default-features=false,features=["impl-cgmath"]}```
#[cfg(feature = "impl-cgmath")]
pub mod cgmath_2d;

/// This module is optional. To use if you must include the feature ```impl-cgmath``` in your ```Cargo.toml``` file.
///
/// By default all features are enabled (this is a work-around to enable the automatic rust doc).
/// If you want to only enable the ```impl-cgmath``` feature you must first disable the default features with ```default-features=false```.
///
/// Cargo.toml example: ```linestring = {version="<current version>",default-features=false,features=["impl-cgmath"]}```
#[cfg(feature = "impl-cgmath")]
pub mod cgmath_3d;

/// This module is optional. To use if you must include the feature ```impl-mint``` in your ```Cargo.toml``` file.
///
/// By default all features are enabled (this is a work-around to enable the automatic rust doc).
/// If you want to only enable the ```impl-mint``` feature you must first disable the default features with ```default-features=false```.
///
/// Cargo.toml example: ```linestring = {version="<current version>",default-features=false,features=["impl-mint"]}```
#[cfg(feature = "impl-mint")]
pub mod mint_2d;
/// This module is optional. To use if you must include the feature ```impl-mint``` in your ```Cargo.toml``` file.
///
/// By default all features are enabled (this is a work-around to enable the automatic rust doc).
/// If you want to only enable the ```impl-mint``` feature you must first disable the default features with ```default-features=false```.
///
/// Cargo.toml example: ```linestring = {version="<current version>",default-features=false,features=["impl-mint"]}```
#[cfg(feature = "impl-mint")]
pub mod mint_3d;

/// This module is optional. To use if you must include the feature ```impl-vecmath``` or ```impl-vec``` in your ```Cargo.toml``` file.
///
/// By default all features are enabled (this is a work-around to enable the automatic rust doc).
/// If you want to only enable the ```impl-vecmath``` or ```impl-vecmath``` features you must first disable the default features with ```default-features=false```.
///
/// Cargo.toml example: ```linestring = {version="<current version>",default-features=false,features=["impl-vecmath"]}```
///
/// Cargo.toml example: ```linestring = {version="<current version>",default-features=false,features=["impl-vec"]}```
#[cfg(any(feature = "impl-vec", feature = "impl-vecmath"))]
pub mod vec_2d;
/// This module is optional. To use if you must include the feature ```impl-vecmath``` or ```impl-vec``` in your ```Cargo.toml``` file.
///
/// By default all features are enabled (this is a work-around to enable the automatic rust doc).
/// If you want to only enable the ```impl-vecmath``` or ```impl-vecmath``` features you must first disable the default features with ```default-features=false```.
///
/// Cargo.toml example: ```linestring = {version="<current version>",default-features=false,features=["impl-vecmath"]}```
///
/// Cargo.toml example: ```linestring = {version="<current version>",default-features=false,features=["impl-vec"]}```
#[cfg(any(feature = "impl-vec", feature = "impl-vecmath"))]
pub mod vec_3d;

/// This module is optional. To use if you must include the feature ```impl-vecmath``` in your ```Cargo.toml``` file.
///
/// By default all features are enabled (this is a work-around to enable the automatic rust doc).
/// If you want to only enable the ```impl-vecmath``` feature you must first disable the default features with ```default-features=false```.
///
/// Cargo.toml example: ```linestring = {version="<current version>",default-features=false,features=["impl-vecmath"]}```
#[cfg(feature = "impl-vecmath")]
pub mod vecmath_3d;
