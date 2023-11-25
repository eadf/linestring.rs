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

mod impls;

use crate::LinestringError;
use itertools::Itertools;
use std::{collections, fmt, fs, hash::Hash, io, io::Write, path::Path};
use vector_traits::{
    approx::{ulps_eq, AbsDiffEq, UlpsEq},
    num_traits::{real::Real, AsPrimitive},
    GenericScalar, GenericVector3, HasXY,
};

pub(crate) mod simplify;
#[cfg(test)]
mod tests;

/// Placeholder for different 3d shapes
pub enum Shape3d<T: GenericVector3> {
    Line(Line3<T>),
    Linestring(Vec<T>),
    ParabolicArc(crate::linestring_2d::VoronoiParabolicArc<T::Vector2>),
}

/// Axis aligned planes, used to describe how imported 'flat' data is arranged in space
#[allow(clippy::upper_case_acronyms)]
#[derive(fmt::Debug, Copy, Clone, Eq, PartialEq)]
pub enum Plane {
    XY,
    XZ,
    YZ,
}

impl Plane {
    #[inline(always)]
    /// Try to figure out what axes defines the plane.
    /// If the AABB delta of one axis (a) is virtually nothing compared to
    /// the widest axis (b) while the third axis (c) is comparable to (b)
    /// by some fraction, we assume that that (a) isn't part of the plane.
    ///
    /// It's not possible to compare to zero exactly because blender
    /// leaves some decimal in coordinates that's suppose to be zero.
    pub fn get_plane<T: GenericVector3>(aabb: Aabb3<T>) -> Option<Self> {
        Plane::get_plane_relaxed(
            aabb,
            T::Scalar::default_epsilon(),
            T::Scalar::default_max_ulps(),
        )
    }

    /// Try to figure out what axes defines the plane.
    /// If the AABB delta of one axis (a) is virtually nothing compared to
    /// the widest axis (b) while the third axis (c) is comparable to (b)
    /// by some fraction, we assume that that (a) isn't part of the plane.
    ///
    /// It's not possible to compare to zero exactly because blender
    /// leaves some decimal in coordinates that's suppose to be zero.
    pub fn get_plane_relaxed<T: GenericVector3>(
        aabb: Aabb3<T>,
        epsilon: T::Scalar,
        max_ulps: u32,
    ) -> Option<Plane> {
        if let Some(low_bound) = aabb.get_low() {
            if let Some(high_bound) = aabb.get_high() {
                let dx = high_bound.x() - low_bound.x();
                let dy = high_bound.y() - low_bound.y();
                let dz = high_bound.z() - low_bound.z();
                let max_delta = T::Scalar::max(T::Scalar::max(dx, dy), dz);

                let dx = T::Scalar::ZERO.ulps_eq(&(dx / max_delta), epsilon, max_ulps);
                let dy = T::Scalar::ZERO.ulps_eq(&(dy / max_delta), epsilon, max_ulps);
                let dz = T::Scalar::ZERO.ulps_eq(&(dz / max_delta), epsilon, max_ulps);

                if dx && !dy && !dz {
                    return Some(Plane::YZ);
                }
                if dy && !dx && !dz {
                    return Some(Plane::XZ);
                }
                if dz && !dx && !dy {
                    return Some(Plane::XY);
                }
            }
        }
        None
    }

    /// Copy this Point2 into a Point3, populating the axes defined by 'plane'
    /// An axis will always remain in x,y,z order.
    /// `Plane::XY`: `Point2(XY)` -> `Point3(XY0)`
    /// `Plane::XZ`: `Point2(XY)` -> `Point2(X0Y)`
    /// `Plane::YZ`: `Point2(XY)` -> `Point2(0XY)`
    /// That way the operation is reversible (with regards to axis positions).
    #[inline(always)]
    pub fn point_to_3d<T: GenericVector3>(&self, point: T::Vector2) -> T {
        match self {
            Plane::XY => T::new_3d(point.x(), point.y(), T::Scalar::ZERO),
            Plane::XZ => T::new_3d(point.x(), T::Scalar::ZERO, point.y()),
            Plane::YZ => T::new_3d(T::Scalar::ZERO, point.x(), point.y()),
        }
    }

    /// Copy this Point2 into a Point3, populating the axes defined by 'plane'
    /// An axis will always remain in x,y,z order.
    /// `Plane::XY`: `Point3(XYZ)` -> `Point2(XY)`
    /// `Plane::XZ`: `Point3(XYZ)` -> `Point2(XZ)`
    /// `Plane::YZ`: `Point3(XYZ)` -> `Point2(YZ)`
    /// That way the operation is reversible (with regards to axis positions).
    #[inline(always)]
    pub fn point_to_2d<T: GenericVector3>(&self, point: T) -> T::Vector2 {
        match self {
            Plane::XY => T::Vector2::new_2d(point.x(), point.y()),
            Plane::XZ => T::Vector2::new_2d(point.x(), point.z()),
            Plane::YZ => T::Vector2::new_2d(point.y(), point.z()),
        }
    }
}

/// A 3d line
#[derive(PartialEq, Eq, Copy, Clone, Hash, fmt::Debug)]
pub struct Line3<T: GenericVector3> {
    pub start: T,
    pub end: T,
}

impl<T: GenericVector3> Line3<T> {
    pub fn new(start: T, end: T) -> Self {
        Self { start, end }
    }

    /// returns (area of a triangle)²*4
    fn triangle_area_squared_times_4(p1: &T, p2: &T, p3: &T) -> T::Scalar {
        let v1_x = p1.x() - p2.x();
        let v1_y = p1.y() - p2.y();
        let v1_z = p1.z() - p2.z();

        let v2_x = p3.x() - p2.x();
        let v2_y = p3.y() - p2.y();
        let v2_z = p3.z() - p2.z();

        let x = v1_y * v2_z - v2_y * v1_z;
        let y = v1_x * v2_z - v2_x * v1_z;
        let z = v1_x * v2_y - v2_x * v1_y;
        x * x + y * y + z * z
    }

    /// Return a new line containing the transformed points
    pub fn apply<F: Fn(T) -> T>(self, f: F) -> Self {
        Self {
            start: f(self.start),
            end: f(self.end),
        }
    }
}

/// A simple 3d AABB
/// If min_max is none no data has not been assigned yet.
#[derive(PartialEq, Eq, Copy, Clone, Hash, fmt::Debug)]
pub struct Aabb3<T: GenericVector3> {
    min_max: Option<(T, T)>,
}

impl<T: GenericVector3> Aabb3<T> {
    /// updates the aabb with points
    pub fn with_points(points: &[T]) -> Self {
        let mut rv = Self { min_max: None };
        if !points.is_empty() {
            rv.update_with_points(points)
        }
        rv
    }

    /// update the aabb with several points
    pub fn update_with_points(&mut self, points: &[T]) {
        if points.is_empty() {
            return;
        }
        if self.min_max.is_none() {
            self.update_with_point(points[0]);
        }
        let (mut aabb_min, mut aabb_max) = self.min_max.unwrap();
        for point in points.iter() {
            aabb_min.set_x(aabb_min.x().min(point.x()));
            aabb_min.set_y(aabb_min.y().min(point.y()));
            aabb_min.set_z(aabb_min.z().min(point.z()));
            aabb_max.set_x(aabb_max.x().max(point.x()));
            aabb_max.set_y(aabb_max.y().max(point.y()));
            aabb_max.set_z(aabb_max.z().max(point.z()));
        }
        self.min_max = Some((aabb_min, aabb_max));
    }

    /// returns the center point of the AABB if it exists
    pub fn center(&self) -> Option<T> {
        if let Some((low, high)) = self.min_max {
            return Some((low + high) / T::Scalar::TWO);
        }
        None
    }

    /// returns true if this aabb contains a point (inclusive)
    #[inline(always)]
    pub fn contains_point_inclusive(&self, point: T) -> bool {
        if let Some(aabb) = self.min_max {
            (aabb.0.x() < point.x() || ulps_eq!(&aabb.0.x(), &point.x()))
                && (aabb.0.y() < point.y() || ulps_eq!(&aabb.0.y(), &point.y()))
                && (aabb.0.z() < point.z() || ulps_eq!(&aabb.0.z(), &point.z()))
                && (aabb.1.x() > point.x() || ulps_eq!(&aabb.1.x(), &point.x()))
                && (aabb.1.y() > point.y() || ulps_eq!(&aabb.1.y(), &point.y()))
                && (aabb.1.z() > point.z() || ulps_eq!(&aabb.1.z(), &point.z()))
        } else {
            false
        }
    }

    /// Apply an operation over each coordinate.
    pub fn apply<F: Fn(T) -> T>(&mut self, f: &F) {
        if let Some(ref mut min_max) = self.min_max {
            self.min_max = Some((f(min_max.0), f(min_max.1)))
        }
    }
}

struct PriorityDistance<T: GenericVector3> {
    key: T::Scalar,
    index: usize,
}

pub struct WindowIterator<'a, T: GenericVector3>(std::slice::Windows<'a, T>);

impl<'a, T: GenericVector3> WindowIterator<'a, T> {
    // TODO: add is_empty once the unstable feature "exact_size_is_empty" has transitioned
    #[inline(always)]
    pub fn is_empty(&self) -> bool {
        self.0.len() == 0
    }

    #[inline(always)]
    pub fn len(&self) -> usize {
        self.0.len()
    }
}

pub struct ChunkIterator<'a, T: GenericVector3>(std::slice::ChunksExact<'a, T>);

impl<'a, T: GenericVector3> ChunkIterator<'a, T> {
    // TODO: add is_empty once the unstable feature "exact_size_is_empty" has transitioned
    #[inline(always)]
    pub fn is_empty(&self) -> bool {
        self.0.len() == 0
    }

    #[inline(always)]
    pub fn len(&self) -> usize {
        self.0.len()
    }

    #[must_use]
    #[inline(always)]
    pub fn remainder(&self) -> &'a [T] {
        self.0.remainder()
    }
}

pub trait LineString3<T: GenericVector3> {
    /// Copy this linestring3 into a linestring2, keeping the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    fn copy_to_2d(&self, plane: Plane) -> Vec<T::Vector2>;

    /// Returns `true` if the last and first points of the collection are exactly the same,
    /// indicating a closed loop.
    /// Note that an empty linestring is also "closed" since first() and last() object are the
    /// same. I.e. None
    fn is_connected(&self) -> bool;

    //// Returns an iterator over consecutive pairs of points in the line string, forming continuous lines.
    /// This iterator is created using `.windows(2)` on the underlying point collection.
    #[must_use = "iterator adaptors are lazy and do nothing unless consumed"]
    fn window_iter(&self) -> WindowIterator<'_, T>;

    /// Returns an iterator over pairs of points in the line string, forming disconnected edges.
    /// This iterator is created using `.chunks_exact(2)` on the underlying point collection.
    #[must_use = "iterator adaptors are lazy and do nothing unless consumed"]
    fn chunk_iter(&self) -> ChunkIterator<'_, T>;

    /// Simplify using Ramer–Douglas–Peucker algorithm adapted for 3d
    fn simplify_rdp(&self, distance_predicate: T::Scalar) -> Self;

    /// Simplify using Visvalingam–Whyatt algorithm adapted for 3d.
    /// This algorithm will delete 'points_to_delete' of points from the polyline with the smallest
    /// area defined by one point and it's neighbours.
    fn simplify_vw(&self, points_to_delete: usize) -> Self;

    /// Apply an operation over each coordinate.
    /// Useful when you want to round the values of the coordinates.
    fn apply<F>(&mut self, f: &F)
    where
        F: Fn(T) -> T;

    /// Creates an iterator that generates intermediate points on a line segment
    /// with steps shorter or equal to the specified distance.
    ///
    /// The iterator ensures that the original points from the line segment are always included.
    /// Each step along the line segment is of equal or shorter length than the specified distance.
    ///
    /// # Parameters
    ///
    /// - `distance`: The desired maximum distance between generated points.
    ///
    /// # Returns
    ///
    /// Returns a `DiscreteLineSegmentIterator` that iterates over the generated points.
    fn discretize(&self, distance: T::Scalar) -> DiscreteLineSegmentIterator<'_, T>;
}

impl<T: GenericVector3> LineString3<T> for Vec<T> {
    /// Copy this linestring3 into a linestring2, keeping the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    fn copy_to_2d(&self, plane: Plane) -> Vec<T::Vector2> {
        match plane {
            Plane::XY => self
                .iter()
                .map(|p3d| T::Vector2::new_2d(p3d.x(), p3d.y()))
                .collect(),
            Plane::XZ => self
                .iter()
                .map(|p3d| T::Vector2::new_2d(p3d.x(), p3d.z()))
                .collect(),
            Plane::YZ => self
                .iter()
                .map(|p3d| T::Vector2::new_2d(p3d.z(), p3d.y()))
                .collect(),
        }
    }

    /// Returns `true` if the last and first points of the collection are exactly the same,
    /// indicating a closed loop.
    /// Note that an empty linestring is also "closed" since first() and last() object are the
    /// same. I.e. None
    fn is_connected(&self) -> bool {
        self.is_empty() || self.first().unwrap() == self.last().unwrap()
    }

    //// Returns an iterator over consecutive pairs of points in the line string, forming continuous lines.
    /// This iterator is created using `.windows(2)` on the underlying point collection.
    #[must_use = "iterator adaptors are lazy and do nothing unless consumed"]
    fn window_iter(&self) -> WindowIterator<'_, T> {
        WindowIterator(self.windows(2))
    }

    /// Returns an iterator over pairs of points in the line string, forming disconnected edges.
    /// This iterator is created using `.chunks_exact(2)` on the underlying point collection.
    #[must_use = "iterator adaptors are lazy and do nothing unless consumed"]
    fn chunk_iter(&self) -> ChunkIterator<'_, T> {
        ChunkIterator(self.chunks_exact(2))
    }

    /// Simplify using Ramer–Douglas–Peucker algorithm adapted for 3d
    #[inline(always)]
    fn simplify_rdp(&self, distance_predicate: T::Scalar) -> Self {
        simplify::simplify_rdp(self, distance_predicate)
    }

    /// Simplify using Visvalingam–Whyatt algorithm adapted for 3d.
    /// This algorithm will delete 'points_to_delete' of points from the polyline with the smallest
    /// area defined by one point and it's neighbours.
    /// Simplify using Visvalingam–Whyatt algorithm. This algorithm will delete 'points_to_delete'
    /// of points from the polyline with the smallest area defined by one point and it's neighbours.
    #[inline(always)]
    fn simplify_vw(&self, points_to_delete: usize) -> Self {
        simplify::simplify_vw(self, points_to_delete)
    }

    fn apply<F: Fn(T) -> T>(&mut self, f: &F) {
        self.iter_mut().for_each(|v| *v = f(*v));
    }

    /// Creates an iterator that generates intermediate points on a line segment
    /// with steps shorter or equal to the specified distance.
    ///
    /// The iterator ensures that the original points from the line segment are always included.
    /// Each step along the line segment is of equal or shorter length than the specified distance.
    ///
    /// # Examples
    ///
    /// ```
    /// # use vector_traits::glam::Vec3;
    /// use linestring::prelude::LineString3;
    ///
    /// let points = vec![
    ///     Vec3::new(0.5, 0.0, 0.0),
    ///     Vec3::new(1.0, 0.0, 0.0),
    ///     Vec3::new(2.0, 0.0, 0.0),
    ///     Vec3::new(3.0, 0.0, 0.0),
    /// ];
    ///
    /// let distance = 0.8;
    /// let iterator = points.discretize(distance);
    ///
    /// let result: Vec<Vec3> = iterator.collect();
    ///
    /// // Ensure that the result contains the original points and intermediate steps
    /// assert_eq!(result, vec![
    ///     Vec3::new(0.5, 0.0, 0.0),
    ///     Vec3::new(1.0, 0.0, 0.0),
    ///     Vec3::new(1.5, 0.0, 0.0),
    ///     Vec3::new(2.0, 0.0, 0.0),
    ///     Vec3::new(2.5, 0.0, 0.0),
    ///     Vec3::new(3.0, 0.0, 0.0),
    /// ]);
    /// ```
    ///
    /// # Parameters
    ///
    /// - `distance`: The desired maximum distance between generated points.
    ///
    /// # Returns
    ///
    /// Returns a `DiscreteLineSegmentIterator` that iterates over the generated points.
    fn discretize(&self, distance: T::Scalar) -> DiscreteLineSegmentIterator<'_, T> {
        DiscreteLineSegmentIterator::new(self, distance)
    }
}

impl<T: GenericVector3, IT> From<[IT; 2]> for Aabb3<T>
where
    IT: Copy + Into<T>,
{
    fn from(coordinate: [IT; 2]) -> Aabb3<T> {
        Aabb3 {
            min_max: Some((coordinate[0].into(), coordinate[1].into())),
        }
    }
}

impl<T: GenericVector3> From<[T::Scalar; 6]> for Aabb3<T> {
    fn from(coordinate: [T::Scalar; 6]) -> Aabb3<T> {
        Aabb3 {
            min_max: Some((
                T::new_3d(coordinate[0], coordinate[1], coordinate[2]),
                T::new_3d(coordinate[3], coordinate[4], coordinate[5]),
            )),
        }
    }
}

impl<T: GenericVector3> Aabb3<T> {
    pub fn update_aabb(&mut self, aabb: Aabb3<T>) {
        if let Some((min, max)) = aabb.min_max {
            self.update_with_point(min);
            self.update_with_point(max);
        }
    }

    pub fn update_with_point(&mut self, point: T) {
        if self.min_max.is_none() {
            self.min_max = Some((point, point));
            return;
        }
        let (mut aabb_min, mut aabb_max) = self.min_max.take().unwrap();

        if point.x() < aabb_min.x() {
            aabb_min.set_x(point.x());
        }
        if point.y() < aabb_min.y() {
            aabb_min.set_y(point.y());
        }
        if point.z() < aabb_min.z() {
            aabb_min.set_z(point.z());
        }
        if point.x() > aabb_max.x() {
            aabb_max.set_x(point.x());
        }
        if point.y() > aabb_max.y() {
            aabb_max.set_y(point.y());
        }
        if point.z() > aabb_max.z() {
            aabb_max.set_z(point.z());
        }
        self.min_max = Some((aabb_min, aabb_max));
    }

    pub fn get_high(&self) -> Option<T> {
        if let Some((_, _high)) = self.min_max {
            return Some(_high);
        }
        None
    }

    pub fn get_low(&self) -> Option<T> {
        if let Some((_low, _)) = self.min_max {
            return Some(_low);
        }
        None
    }

    /// returns true if this aabb entirely contains/engulfs 'other' (inclusive)
    #[inline(always)]
    pub fn contains_aabb(&self, other: &Self) -> bool {
        if let Some(self_aabb) = other.min_max {
            if let Some(o_aabb) = other.min_max {
                return Self::contains_point_(self_aabb, o_aabb.0)
                    && Self::contains_point_(self_aabb, o_aabb.1);
            }
        }
        false
    }

    /// returns true if this aabb entirely contains/engulfs a line (inclusive)
    #[inline(always)]
    pub fn contains_line(&self, line: &Line3<T>) -> bool {
        if let Some(self_aabb) = self.min_max {
            return Self::contains_point_(self_aabb, line.start)
                && Self::contains_point_(self_aabb, line.end);
        }
        false
    }

    /// returns true if this aabb contains a point (inclusive)
    #[inline(always)]
    pub fn contains_point(&self, point: T) -> bool {
        if let Some(self_aabb) = self.min_max {
            return Self::contains_point_(self_aabb, point);
        }
        false
    }

    /// returns true if aabb contains a point (inclusive)
    #[inline(always)]
    fn contains_point_(aabb: (T, T), point: T) -> bool {
        aabb.0.x() <= point.x()
            && aabb.0.y() <= point.y()
            && aabb.0.z() <= point.z()
            && aabb.1.x() >= point.x()
            && aabb.1.y() >= point.y()
            && aabb.1.z() >= point.z()
    }
}

#[allow(clippy::many_single_char_names)]
#[inline(always)]
fn cross_abs_squared<T: GenericVector3>(a: T, b: T) -> T::Scalar {
    let x = a.y() * b.z() - a.z() * b.y();
    let y = a.z() * b.x() - a.x() * b.z();
    let z = a.x() * b.y() - a.y() * b.x();
    x * x + y * y + z * z
}

#[inline(always)]
/// The distance between the line a->b to the point p is the same as
/// distance = |(a-p)×(a-b)|/|a-b|
/// <https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Another_vector_formulation>
/// Make sure to *not* call this function with a-b==0
/// This function returns the distance²
pub fn distance_to_line_squared<T: GenericVector3>(l0: T, l1: T, p: T) -> T::Scalar {
    let l0_sub_l1 = l0 - l1;
    let l0_sub_p = l0 - p;
    cross_abs_squared(l0_sub_p, l0_sub_l1)
        / (l0_sub_l1.x() * l0_sub_l1.x()
            + l0_sub_l1.y() * l0_sub_l1.y()
            + l0_sub_l1.z() * l0_sub_l1.z())
}

/// Same as `distance_to_line_squared<T>` but it can be called when a-b might be 0.
/// It's a little slower because it does the a==b test
#[inline(always)]
pub fn distance_to_line_squared_safe<T: GenericVector3>(l0: T, l1: T, p: T) -> T::Scalar {
    if l0.is_ulps_eq(
        l1,
        T::Scalar::default_epsilon(),
        T::Scalar::default_max_ulps(),
    ) {
        // give the point-to-point answer if the segment is a point
        l0.distance_sq(p)
    } else {
        let l0_sub_l1 = l0 - l1;
        let l0_sub_p = l0 - p;
        cross_abs_squared(l0_sub_p, l0_sub_l1)
            / (l0_sub_l1.x() * l0_sub_l1.x()
                + l0_sub_l1.y() * l0_sub_l1.y()
                + l0_sub_l1.z() * l0_sub_l1.z())
    }
}

/// Rudimentary save line strings to .obj file function
pub fn save_to_obj_file<T: GenericVector3>(
    filename: impl AsRef<Path>,
    object_name: &str,
    lines: Vec<Vec<Line3<T>>>,
) -> Result<(), LinestringError> {
    let mut point_set = collections::HashMap::<String, usize>::new();
    // try to de-duplicate points
    for i in lines.iter() {
        for j in i.iter() {
            let a_str = format!("v {:+e} {:+e} {:+e}", j.start.x(), j.start.y(), j.start.z());
            let len = point_set.len();
            let _ = point_set.entry(a_str).or_insert(len);
            let a_str = format!("v {:+e} {:+e} {:+e}", j.end.x(), j.end.y(), j.end.z());
            let len = point_set.len();
            let _ = point_set.entry(a_str).or_insert(len);
        }
    }
    let mut file = io::BufWriter::new(fs::File::create(filename)?);
    writeln!(file, "o {}", object_name)?;
    for (k, v) in point_set.iter().sorted_unstable_by(|a, b| a.1.cmp(b.1)) {
        writeln!(file, "{} #{}", k, v + 1)?;
    }
    for i in lines.iter() {
        for j in i.iter() {
            let str0 = format!("v {:+e} {:+e} {:+e}", j.start.x(), j.start.y(), j.start.z());
            if let Some(p0) = point_set.get(&str0) {
                let str1 = format!("v {:+e} {:+e} {:+e}", j.end.x(), j.end.y(), j.end.z());
                if let Some(p1) = point_set.get(&str1) {
                    writeln!(file, "l {} {}", p0 + 1, p1 + 1)?;
                } else {
                    return Err(LinestringError::InternalError(format!(
                        "HashMap can't find the key:{}",
                        str1
                    )));
                }
            } else {
                return Err(LinestringError::InternalError(format!(
                    "HashMap can't find the key:{}",
                    str0
                )));
            }
        }
    }
    file.flush()?;
    Ok(())
}

pub struct DiscreteLineSegmentIterator<'a, T: GenericVector3> {
    points: &'a Vec<T>,
    distance: T::Scalar,
    current_index: usize,
    substep_index: usize,
    num_substeps: usize,
    step: T,
}

impl<'a, T: GenericVector3> DiscreteLineSegmentIterator<'a, T> {
    fn new(points: &'a Vec<T>, distance: T::Scalar) -> Self {
        DiscreteLineSegmentIterator {
            points,
            distance,
            current_index: 0,
            substep_index: 0,
            num_substeps: 0,
            step: T::new_3d(T::Scalar::ZERO, T::Scalar::ZERO, T::Scalar::ZERO),
        }
    }

    fn initialize_step(&mut self) {
        let start_point = self.points[self.current_index];
        let end_point = self.points[self.current_index + 1];
        let direction = end_point - start_point;
        let segment_length = direction.magnitude();
        if segment_length > T::Scalar::ZERO {
            let num_substeps = (segment_length / self.distance).ceil();
            self.num_substeps = num_substeps.as_();
            self.step = direction / num_substeps;
        } else {
            // Handle the case when segment_length is exactly 0.0
            self.num_substeps = 1;
            self.step = T::new_3d(T::Scalar::ZERO, T::Scalar::ZERO, T::Scalar::ZERO);
        }
    }
}

impl<'a, T: GenericVector3> Iterator for DiscreteLineSegmentIterator<'a, T>
where
    usize: AsPrimitive<<T as HasXY>::Scalar>,
{
    type Item = T;

    fn next(&mut self) -> Option<Self::Item> {
        if self.points.is_empty() {
            return None;
        }
        match self.points.len() - 1 {
            n if self.current_index < n => {
                if self.substep_index == 0 {
                    // Initialize step on a new line segment
                    self.initialize_step();
                }

                if self.substep_index < self.num_substeps {
                    let start_point = self.points[self.current_index];
                    let result = start_point + self.step * (self.substep_index.as_());
                    self.substep_index += 1;
                    return Some(result);
                }

                // Move to the next line segment
                self.current_index += 1;
                self.substep_index = 0;
                self.next()
            }
            n if self.current_index == n => {
                // return the very last point of the list
                self.current_index += 1;
                self.substep_index = 0;
                return self.points.last().copied();
            }
            _ => None,
        }
    }
}
