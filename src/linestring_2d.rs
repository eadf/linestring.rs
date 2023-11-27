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

use crate::{linestring_3d, linestring_3d::Plane, LinestringError};
use std::fmt::Debug;
use vector_traits::{
    approx::*,
    num_traits::{AsPrimitive, Float, One, Zero},
    GenericScalar, GenericVector2, HasXY, HasXYZ,
};

/// Module containing the convex hull calculations
pub mod convex_hull;
pub mod impls;
/// Module containing the intersection calculations but for indexed vertexes
pub mod indexed_intersection;
/// Module containing the intersection calculations
pub mod intersection;
pub(crate) mod simplify;
#[cfg(test)]
mod tests;

/// A 2D polyline representation with some operations.
///
/// The trait `LineString2` provides a number of utility function that allows us to treat a
/// `Vec<GenericVector2>` as a connected line of points.
///
/// When the last and the first point of the line are identical, the line is considered 'connected' or closed.
///
/// **Features**:
/// - **Intersections**:
///   - Detect line intersections.
///   - Detect self-intersections.
///   - Compute ray intersections.
///
/// - **Simplification**:
///   - Line string simplification using both the Ramer-Douglas-Peucker (RDP) and Visvalingam-Whyatt (VW) methods.
///
/// - **Convex Hulls**:
///   - Generate convex hulls using both Gift Wrapping and Graham's Scan algorithms.
///   - Inclusion tests for points, AABBs, and other convex hulls, both in parallel and single-threaded versions.
///
/// In its essence, this structure provides foundational utilities for handling and manipulating
/// poly-lines in a 2D space.
///
pub trait LineString2<T: GenericVector2> {
    /// returns true if fist element == last element
    fn is_connected(&self) -> bool;

    /// Returns the number of points in the line.
    /// If the linestring is connected it does not count the last point
    fn point_count(&self) -> usize;

    /// Returns true if the lines are self intersecting
    /// If number of points < 10 then the intersections are tested using brute force O(n²)
    /// If more than that a sweep-line algorithm is used O(n*log(n)+i*log(n))
    fn is_self_intersecting(&self) -> Result<bool, LinestringError>;

    /// This intersection method returns the first intersection it finds.
    fn intersection(&self, other: &Line2<T>) -> Option<Intersection<T>>;

    /// intersection method that returns two possible intersection points
    /// Use this when you know there can only be two intersection points (e.g. a convex hulls)
    fn find_two_intersections(&self, other: &Line2<T>) -> (Option<T>, Option<T>);

    /// intersection method that returns all possible intersection points
    fn find_all_intersections(&self, other: &Line2<T>) -> Vec<Intersection<T>>;

    /// tests if a ray intersect an edge from p0 to p1
    /// It returns the smallest `t` it can find
    fn ray_edge_intersection(origin: T, ray_dir: T, p0: T, p1: T) -> Option<T::Scalar>;

    /// This method returns the closest ray intersections with the Linestring.
    /// This might return the origin position if the position is already on a line segment
    fn closest_ray_intersection(&self, ray_dir: T, origin: T) -> Option<T>;

    /// returns true if this and the other objects are the same.
    /// float equality is determined by ulps_eq!()
    fn approx_eq(&self, other: &Self) -> bool;

    /// Returns an iterator over consecutive pairs of points in the line string, forming continuous lines.
    /// This iterator is created using `.windows(2)` on the underlying point collection.
    #[must_use = "iterator adaptors are lazy and do nothing unless consumed"]
    fn window_iter(&self) -> WindowIterator<'_, T>;

    /// Returns an iterator over pairs of points in the line string, forming disconnected edges.
    /// This iterator is created using `.chunks_exact(2)` on the underlying point collection.
    #[must_use = "iterator adaptors are lazy and do nothing unless consumed"]
    fn chunk_iter(&self) -> ChunkIterator<'_, T>;

    /// Copy this Linestring2 into a Linestring3, populating the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    fn copy_to_3d(&self, plane: Plane) -> Vec<T::Vector3>;

    /// Simplify using Ramer–Douglas–Peucker algorithm
    fn simplify_rdp(&self, distance_predicate: T::Scalar) -> Self;

    /// Simplify using Visvalingam–Whyatt algorithm. This algorithm will delete 'points_to_delete'
    /// of points from the polyline with the smallest area defined by one point and it's neighbours.
    fn simplify_vw(&self, points_to_delete: usize) -> Self;

    /// Returns true if self is a convex hull and it contains the 'b' point (inclusive)
    /// This function is single threaded.
    /// Will return unpredictable results if self is not a convex hull.
    fn contains_point_inclusive(&self, p: T) -> bool;

    /// Returns true if self is a convex hull and it contains the 'b' point (exclusive)
    /// This function is single threaded.
    /// Will return unpredictable results if self is not a convex hull.
    fn contains_point_exclusive(&self, p: T) -> bool;

    /// Generates the convex hull of the points contained within
    fn convex_hull(&self) -> Result<Vec<T>, LinestringError>;

    /// Generates the convex hull of the points contained within using a multi-threaded method.
    /// `per_thread_chunk_size` is the number of vertices in each multi-threaded chunk
    fn convex_hull_par(&self, per_thread_chunk_size: usize) -> Result<Vec<T>, LinestringError>;

    /// Apply an operation over each coordinate.
    /// Useful when you, for example, want to round the values of the coordinates.
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

/// A 2d line
#[derive(Copy, Clone)]
pub struct Line2<T> {
    pub start: T,
    pub end: T,
}

impl<T: GenericVector2> Line2<T> {
    pub fn new(start: T, end: T) -> Self {
        Self { start, end }
    }

    /// Returns the intersection point of two line segments.
    ///
    /// This method calculates the intersection of two line segments, considering various cases:
    /// - Non-intersecting segments.
    /// - Intersecting segments at a single point.
    /// - Segments that overlap each other.
    /// - Endpoint-to-endpoint intersections.
    ///
    /// The method can also handle cases where one or both of the line segments are reduced to a point.
    ///
    /// The algorithm implemented here is inspired by and adapted from
    /// [this explanation on StackOverflow](https://stackoverflow.com/a/565282).
    ///
    /// # Examples
    ///
    /// ```
    /// # use vector_traits::approx::*;
    /// # use linestring::linestring_2d::{Line2,Intersection};
    /// # use vector_traits::glam::vec2;
    /// let segment1 = Line2::new(vec2(0.0, 0.0), vec2(5.0, 0.0));
    /// let segment2 = Line2::new(vec2(1.0, 0.0), vec2(3.0, 0.0));
    /// if let Some(Intersection::Overlap(inter))= segment1.intersection_point(segment2) {
    ///    assert!(ulps_eq!(inter, Line2::new(vec2(1.0, 0.0), vec2(3.0, 0.0))));
    /// } else { panic!() }
    /// ```
    ///
    /// # Returns
    ///
    /// - `Some(Intersection::Intersection(point))` if there's a single intersection point.
    /// - `Some(Intersection::Overlap(segment))` if the segments overlap each other.
    /// - `None` if the segments don't intersect.
    ///
    /// # Note
    ///
    /// The method may return overlapping segments in a different order (flipped start and end points)
    /// compared to input. This is because a line segment from A to B is considered identical to a segment
    /// from B to A for the purposes of this function.
    pub fn intersection_point(self, other: Self) -> Option<Intersection<T>> {
        // Check if either segment is a point
        if self.start.is_ulps_eq(
            self.end,
            T::Scalar::default_epsilon(),
            T::Scalar::default_max_ulps(),
        ) {
            return intersect_line_point(other, self.start);
        } else if other.start.is_ulps_eq(
            other.end,
            T::Scalar::default_epsilon(),
            T::Scalar::default_max_ulps(),
        ) {
            return intersect_line_point(self, other.start);
        }

        // AABB tests
        let self_min_x = self.start.x().min(self.end.x());
        let self_max_x = self.start.x().max(self.end.x());
        let self_min_y = self.start.y().min(self.end.y());
        let self_max_y = self.start.y().max(self.end.y());

        let other_min_x = other.start.x().min(other.end.x());
        let other_max_x = other.start.x().max(other.end.x());
        let other_min_y = other.start.y().min(other.end.y());
        let other_max_y = other.start.y().max(other.end.y());

        if self_max_x < other_min_x
            || self_min_x > other_max_x
            || self_max_y < other_min_y
            || self_min_y > other_max_y
        {
            return None;
        }

        let mut p = self.start;
        let mut q = other.start;
        let mut r = self.end - p;
        let mut s = other.end - q;

        let r_cross_s = r.perp_dot(s);

        // If r × s = 0 then the two lines are parallel
        if ulps_eq!(r_cross_s, T::Scalar::ZERO) {
            // sort the edge start and end points
            if Self::precedes(self.start, self.end) {
                p = self.end;
                r = -r;
            }
            if Self::precedes(other.start, other.end) {
                q = other.end;
                s = -s;
            }

            let q_minus_p = q - p;
            let q_minus_p_cross_r = q_minus_p.perp_dot(r);

            ulps_eq!(q_minus_p_cross_r, T::Scalar::ZERO).then(|| {
                let r_dot_r = r.dot(r);
                let t0 = q_minus_p.dot(r) / r_dot_r;
                let t1 = t0 + s.dot(r) / r_dot_r;

                let overlap_start = p + r * t0.max(T::Scalar::ZERO);
                let overlap_end = p + r * t1.min(T::Scalar::ONE);

                if overlap_start == overlap_end {
                    Intersection::Intersection(overlap_start)
                } else {
                    Intersection::Overlap(Line2::new(overlap_start, overlap_end))
                }
            })
        } else {
            let q_minus_p = q - p;
            let t = q_minus_p.perp_dot(s) / r_cross_s;
            let u = q_minus_p.perp_dot(r) / r_cross_s;

            if (T::Scalar::ZERO..=T::Scalar::ONE).contains(&t)
                && (T::Scalar::ZERO..=T::Scalar::ONE).contains(&u)
            {
                Some(Intersection::Intersection(p + r * t))
            } else {
                None
            }
        }
    }

    /// an artificial ordering of vectors
    #[inline(always)]
    fn precedes(a: T, b: T) -> bool {
        if a.x() > b.x() {
            true
        } else {
            a.y() > b.y() && ulps_eq!(a.x(), b.x())
        }
    }

    /// Intersection test for lines known to be connected by a middle point.
    /// This function will *not* report the middle-point as an intersection.
    /// Todo: how to handle start==middle||middle==end
    fn intersection_point3(
        start: T,
        middle: T,
        end: T,
    ) -> Result<Option<Intersection<T>>, LinestringError> {
        let middle_sub_start = T::new_2d(middle.x() - start.x(), middle.y() - start.x());
        let middle_sub_end = T::new_2d(middle.x() - end.x(), middle.y() - end.y());
        let start_is_vertical = ulps_eq!(middle_sub_start.x(), T::Scalar::ZERO);
        let end_is_vertical = ulps_eq!(middle_sub_end.x(), T::Scalar::ZERO);

        if start_is_vertical && end_is_vertical {
            // both lines are vertical
            return if middle_sub_start.y().is_sign_negative()
                && !middle_sub_end.y().is_sign_negative()
            {
                // opposite direction
                Ok(None)
            } else {
                // pick the shortest vector for overlap point
                if (middle_sub_start.x() * middle_sub_start.x()
                    + middle_sub_start.y() * middle_sub_start.y())
                    < (middle_sub_end.x() * middle_sub_end.x()
                        + middle_sub_end.y() * middle_sub_end.y())
                {
                    Ok(Some(Intersection::Overlap(Line2 { start, end: middle })))
                } else {
                    Ok(Some(Intersection::Overlap(Line2 { start: middle, end })))
                }
            };
        } else if start_is_vertical || end_is_vertical {
            return Ok(None);
        }

        // both lines should now be non-vertical, we can compare their slope
        let start_slope = middle_sub_start.x() / middle_sub_start.y();
        let end_slope = middle_sub_end.x() / middle_sub_end.y();

        if !ulps_eq!(&start_slope, &end_slope) {
            return Ok(None);
        }

        // Slope identical, pick the shortest vector for overlap point
        if (middle_sub_start.x() * middle_sub_start.x()
            + middle_sub_start.y() * middle_sub_start.y())
            < (middle_sub_end.x() * middle_sub_end.x() + middle_sub_end.y() * middle_sub_end.y())
        {
            Ok(Some(Intersection::Overlap(Line2 { start, end: middle })))
        } else {
            Ok(Some(Intersection::Overlap(Line2 { start: middle, end })))
        }
    }

    /// returns (area of a triangle)²*4
    /// <https://en.wikipedia.org/wiki/Visvalingam–Whyatt_algorithm#Algorithm>
    fn triangle_area_squared_times_4(p1: T, p2: T, p3: T) -> T::Scalar {
        let area = p1.x() * p2.y() + p2.x() * p3.y() + p3.x() * p1.y()
            - p1.x() * p3.y()
            - p2.x() * p1.y()
            - p3.x() * p2.y();
        area * area
    }

    /// Copy this lines2 into a line3, populating the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    pub fn copy_to_3d(&self, plane: Plane) -> linestring_3d::Line3<T::Vector3> {
        linestring_3d::Line3::new(
            plane.point_to_3d::<T::Vector3>(self.start),
            plane.point_to_3d::<T::Vector3>(self.end),
        )
    }
}

/// A parabolic arc as used in <https://github.com/eadf/boostvoronoi.rs>
/// This struct contains the parameters for the arc + the functionality to convert it to a
/// LineString2 or LineString3.
///
/// This parabola describes the equidistant curve between a point and a segment starting
/// at 'start_point' and ending at 'end_point'
/// This also mean that the distance between 'start_point'<->cell_point and
/// 'end_point'<->cell_point must be the same distance as 'start_point'<->segment &
/// 'end_point'<->segment
#[derive(Clone, Debug)]
pub struct VoronoiParabolicArc<T: GenericVector2> {
    // input geometry of voronoi graph
    pub segment: Line2<T>,
    pub cell_point: T,

    // vertex points in voronoi diagram. Aka edge start and end points. (Also called circle events)
    pub start_point: T,
    pub end_point: T,
}

impl<T: GenericVector2> VoronoiParabolicArc<T> {
    pub fn new(segment: Line2<T>, cell_point: T, start_point: T, end_point: T) -> Self {
        Self {
            segment,
            cell_point,
            start_point,
            end_point,
        }
    }

    /// Convert this parable abstraction into discrete line segment sample points.
    /// All of this code is ported from C++ boost 1.75.0
    /// <https://www.boost.org/doc/libs/1_75_0/libs/polygon/doc/voronoi_main.htm>
    pub fn discretize_2d(&self, max_dist: T::Scalar) -> Vec<T> {
        let mut rv = vec![self.start_point];

        // Apply the linear transformation to move start point of the segment to
        // the point with coordinates (0, 0) and the direction of the segment to
        // coincide the positive direction of the x-axis.
        let segm_vec_x = self.segment.end.x() - self.segment.start.x();
        let segm_vec_y = self.segment.end.y() - self.segment.start.y();
        let sqr_segment_length = segm_vec_x * segm_vec_x + segm_vec_y * segm_vec_y;

        // Compute x-coordinates of the endpoints of the edge
        // in the transformed space.
        let projection_start =
            Self::point_projection(self.start_point, self.segment) * sqr_segment_length;
        let projection_end =
            Self::point_projection(self.end_point, self.segment) * sqr_segment_length;

        // Compute parabola parameters in the transformed space.
        // Parabola has next representation:
        // f(x) = ((x-rot_x)^2 + rot_y^2) / (2.0*rot_y).
        let point_vec_x = self.cell_point.x() - self.segment.start.x();
        let point_vec_y = self.cell_point.y() - self.segment.start.y();
        let rot_x = segm_vec_x * point_vec_x + segm_vec_y * point_vec_y;
        let rot_y = segm_vec_x * point_vec_y - segm_vec_y * point_vec_x;

        // Use stack to avoid recursion.
        let mut point_stack = vec![projection_end];
        let mut cur_x = projection_start;
        let mut cur_y = Self::parabola_y(cur_x, rot_x, rot_y);

        // Adjust max_dist parameter in the transformed space.
        let max_dist_transformed = max_dist * max_dist * sqr_segment_length;
        while !point_stack.is_empty() {
            let new_x = point_stack[point_stack.len() - 1]; // was .top();
            let new_y = Self::parabola_y(new_x, rot_x, rot_y);

            // Compute coordinates of the point of the parabola that is
            // furthest from the current line segment.
            let mid_x = (new_y - cur_y) / (new_x - cur_x) * rot_y + rot_x;
            let mid_y = Self::parabola_y(mid_x, rot_x, rot_y);

            // Compute maximum distance between the given parabolic arc
            // and line segment that discretize it.
            let mut dist = (new_y - cur_y) * (mid_x - cur_x) - (new_x - cur_x) * (mid_y - cur_y);
            #[allow(clippy::suspicious_operation_groupings)]
            {
                dist = dist * dist
                    / ((new_y - cur_y) * (new_y - cur_y) + (new_x - cur_x) * (new_x - cur_x));
            }
            if dist <= max_dist_transformed {
                // Distance between parabola and line segment is less than max_dist.
                let _ = point_stack.pop();
                let inter_x = (segm_vec_x * new_x - segm_vec_y * new_y) / sqr_segment_length
                    + self.segment.start.x();
                let inter_y = (segm_vec_x * new_y + segm_vec_y * new_x) / sqr_segment_length
                    + self.segment.start.y();
                let p = T::new_2d(inter_x, inter_y);
                rv.push(p);
                cur_x = new_x;
                cur_y = new_y;
            } else {
                point_stack.push(mid_x);
            }
        }

        // Update last point.
        let last_position = rv.len() - 1;
        rv[last_position] = self.end_point;
        rv
    }

    /// Convert this parable abstraction into a single straight line
    pub fn discretize_3d_straight_line(&self) -> Vec<T::Vector3> {
        let mut rv = Vec::default();
        let distance =
            -distance_to_line_squared_safe(self.segment.start, self.segment.end, self.start_point)
                .sqrt();
        rv.push(T::Vector3::new_3d(
            self.start_point.x(),
            self.start_point.y(),
            distance,
        ));
        let distance = -self.end_point.distance(self.cell_point);
        rv.push(T::Vector3::new_3d(
            self.end_point.x(),
            self.end_point.y(),
            distance,
        ));
        rv
    }

    /// Convert this parable abstraction into discrete line segment sample points.
    /// The Z component of the coordinates is the constant distance from the edge to point and
    /// line segment (should be the same value)
    ///
    /// All of this code is ported from C++ boost 1.75.0
    /// <https://www.boost.org/doc/libs/1_75_0/libs/polygon/doc/voronoi_main.htm>
    pub fn discretize_3d(&self, max_dist: T::Scalar) -> Vec<T::Vector3> {
        let mut rv = Vec::default();
        let z_comp = -self.start_point.distance(self.cell_point);
        rv.push(T::Vector3::new_3d(
            self.start_point.x(),
            self.start_point.y(),
            z_comp,
        ));

        let z_comp = -self.end_point.distance(self.cell_point);
        // todo, don't insert end_point and then pop it again a few lines later..
        rv.push(T::Vector3::new_3d(
            self.end_point.x(),
            self.end_point.y(),
            z_comp,
        ));

        // Apply the linear transformation to move start point of the segment to
        // the point with coordinates (0, 0) and the direction of the segment to
        // coincide the positive direction of the x-axis.
        let segm_vec_x = self.segment.end.x() - self.segment.start.x();
        let segm_vec_y = self.segment.end.y() - self.segment.start.y();
        let sqr_segment_length = segm_vec_x * segm_vec_x + segm_vec_y * segm_vec_y;

        // Compute x-coordinates of the endpoints of the edge
        // in the transformed space.
        let projection_start =
            sqr_segment_length * Self::point_projection_3d(&rv[0], &self.segment);
        let projection_end = sqr_segment_length * Self::point_projection_3d(&rv[1], &self.segment);

        // Compute parabola parameters in the transformed space.
        // Parabola has next representation:
        // f(x) = ((x-rot_x)^2 + rot_y^2) / (2.0*rot_y).
        let point_vec_x = self.cell_point.x() - self.segment.start.x();
        let point_vec_y = self.cell_point.y() - self.segment.start.y();
        let rot_x = segm_vec_x * point_vec_x + segm_vec_y * point_vec_y;
        let rot_y = segm_vec_x * point_vec_y - segm_vec_y * point_vec_x;

        // Save the last point.
        let last_point = (*rv)[1];
        let _ = rv.pop();

        // Use stack to avoid recursion.
        let mut point_stack = vec![projection_end];
        let mut cur_x = projection_start;
        let mut cur_y = Self::parabola_y(cur_x, rot_x, rot_y);

        // Adjust max_dist parameter in the transformed space.
        let max_dist_transformed = max_dist * max_dist * sqr_segment_length;
        while !point_stack.is_empty() {
            let new_x = point_stack[point_stack.len() - 1]; // was .top();
            let new_y = Self::parabola_y(new_x, rot_x, rot_y);

            // Compute coordinates of the point of the parabola that is
            // furthest from the current line segment.
            let mid_x = (new_y - cur_y) / (new_x - cur_x) * rot_y + rot_x;
            let mid_y = Self::parabola_y(mid_x, rot_x, rot_y);

            // Compute maximum distance between the given parabolic arc
            // and line segment that discretize it.
            let mut dist = (new_y - cur_y) * (mid_x - cur_x) - (new_x - cur_x) * (mid_y - cur_y);
            #[allow(clippy::suspicious_operation_groupings)]
            {
                dist = dist * dist
                    / ((new_y - cur_y) * (new_y - cur_y) + (new_x - cur_x) * (new_x - cur_x));
            }
            if dist <= max_dist_transformed {
                // Distance between parabola and line segment is less than max_dist.
                let _ = point_stack.pop();
                let inter_x = (segm_vec_x * new_x - segm_vec_y * new_y) / sqr_segment_length
                    + self.segment.start.x();
                let inter_y = (segm_vec_x * new_y + segm_vec_y * new_x) / sqr_segment_length
                    + self.segment.start.y();
                let z_comp = -T::new_2d(inter_x, inter_y).distance(self.cell_point);
                rv.push(T::Vector3::new_3d(inter_x, inter_y, z_comp));
                cur_x = new_x;
                cur_y = new_y;
            } else {
                point_stack.push(mid_x);
            }
        }

        // Update last point.
        let last_position = rv.len() - 1;
        rv[last_position] = last_point;
        rv
    }

    /// Compute y(x) = ((x - a) * (x - a) + b * b) / (2 * b).
    #[inline(always)]
    #[allow(clippy::suspicious_operation_groupings)]
    fn parabola_y(x: T::Scalar, a: T::Scalar, b: T::Scalar) -> T::Scalar {
        ((x - a) * (x - a) + b * b) / (b + b)
    }

    /// Get normalized length of the distance between:
    ///   1) point projection onto the segment
    ///   2) start point of the segment
    /// Return this length divided by the segment length. This is made to avoid
    /// sqrt computation during transformation from the initial space to the
    /// transformed one and vice versa. The assumption is made that projection of
    /// the point lies between the start-point and endpoint of the segment.
    #[inline(always)]
    fn point_projection(point: T, segment: Line2<T>) -> T::Scalar {
        let segment_vec_x = segment.end.x() - segment.start.x();
        let segment_vec_y = segment.end.y() - segment.start.y();
        let point_vec_x = point.x() - segment.start.x();
        let point_vec_y = point.y() - segment.start.y();
        let sqr_segment_length = segment_vec_x * segment_vec_x + segment_vec_y * segment_vec_y;
        let vec_dot = segment_vec_x * point_vec_x + segment_vec_y * point_vec_y;
        vec_dot / sqr_segment_length
    }

    // exactly the same as get_point_projection but with a Point3 (Z component will be ignored)
    fn point_projection_3d(point: &T::Vector3, segment: &Line2<T>) -> T::Scalar {
        let segment_vec_x = segment.end.x() - segment.start.x();
        let segment_vec_y = segment.end.y() - segment.start.y();
        let point_vec_x = point.x() - segment.start.x();
        let point_vec_y = point.y() - segment.start.y();
        let sqr_segment_length = segment_vec_x * segment_vec_x + segment_vec_y * segment_vec_y;
        let vec_dot = segment_vec_x * point_vec_x + segment_vec_y * point_vec_y;
        vec_dot / sqr_segment_length
    }
}

struct PriorityDistance<T: GenericVector2> {
    key: T::Scalar,
    index: usize,
}

/// An iterator that treats the Vec as a set of connected points, giving an iterator to each `Line2`.
pub struct WindowIterator<'a, T: GenericVector2>(std::slice::Windows<'a, T>);

impl<'a, T: GenericVector2> WindowIterator<'a, T> {
    // TODO: add is_empty once the unstable feature "exact_size_is_empty" has transitioned
    #[inline(always)]
    pub fn is_empty(&self) -> bool {
        self.0.len() == 0
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }
}

/// An iterator that treats the Vec as a set of disconnected points, giving an iterator to each `Line2`.
pub struct ChunkIterator<'a, T: GenericVector2>(std::slice::ChunksExact<'a, T>);

impl<'a, T: GenericVector2> ChunkIterator<'a, T> {
    // TODO: add is_empty once the unstable feature "exact_size_is_empty" has transitioned
    #[inline(always)]
    pub fn is_empty(&self) -> bool {
        self.0.len() == 0
    }

    #[inline(always)]
    pub fn len(&self) -> usize {
        self.0.len()
    }

    #[inline(always)]
    #[must_use]
    pub fn remainder(&self) -> &'a [T] {
        self.0.remainder()
    }
}

impl<T: GenericVector2> LineString2<T> for Vec<T> {
    /// Returns `true` if the last and first points of the collection are exactly the same,
    /// indicating a closed loop.
    /// Note that an empty linestring is also "closed" since first() and last() object are the
    /// same. I.e. None
    fn is_connected(&self) -> bool {
        self.is_empty() || self.first().unwrap() == self.last().unwrap()
    }

    /// Returns the number of points in the line.
    /// If the linestring is connected it does not count the last point
    fn point_count(&self) -> usize {
        if self.len() > 2 && self.is_connected() {
            self.len() - 1
        } else {
            self.len()
        }
    }

    /// Returns true if the lines are self intersecting
    /// If number of points < 10 then the intersections are tested using brute force O(n²)
    /// If more than that a sweep-line algorithm is used O(n*log(n)+i*log(n))
    fn is_self_intersecting(&self) -> Result<bool, LinestringError> {
        if self.len() <= 2 {
            Ok(false)
        } else if self.len() < 10 {
            //let lines = self.as_lines_iter();
            let iter = self.window_iter();
            let iter_len = iter.len();
            for l0 in iter.enumerate().take(iter_len - 1) {
                for l1 in self.window_iter().enumerate().skip(l0.0 + 1) {
                    if l0.0 == l1.0 {
                        continue;
                    } else if l0.0 + 1 == l1.0 {
                        // consecutive line: l0.0.start <-> l0.0.end_point <-> l2.0.end_point
                        if Line2::intersection_point3(l0.1.start, l0.1.end, l1.1.end)?.is_some() {
                            return Ok(true);
                        }
                    } else if let Some(point) = l0.1.intersection_point(l1.1) {
                        let point = point.single();
                        if (point.is_ulps_eq(
                            l0.1.start,
                            T::Scalar::default_epsilon(),
                            T::Scalar::default_max_ulps(),
                        ) || point.is_ulps_eq(
                            l0.1.end,
                            T::Scalar::default_epsilon(),
                            T::Scalar::default_max_ulps(),
                        )) && (point.is_ulps_eq(
                            l1.1.start,
                            T::Scalar::default_epsilon(),
                            T::Scalar::default_max_ulps(),
                        ) || point.is_ulps_eq(
                            l1.1.end,
                            T::Scalar::default_epsilon(),
                            T::Scalar::default_max_ulps(),
                        )) {
                            continue;
                        } else {
                            return Ok(true);
                        }
                    }
                }
            }
            Ok(false)
        } else {
            let result = intersection::IntersectionData::default()
                .with_ignore_end_point_intersections(true)?
                .with_stop_at_first_intersection(true)?
                .with_lines(self.window_iter())?
                .compute()?;
            Ok(result.len() > 0)
        }
    }

    /// This intersection method returns the first intersection it finds.
    fn intersection(&self, other: &Line2<T>) -> Option<Intersection<T>> {
        for l in self.window_iter() {
            let intersection = l.intersection_point(*other);
            if intersection.is_some() {
                return intersection;
            }
        }
        None
    }

    /// intersection method that returns two possible intersection points
    /// Use this when you know there can only be two intersection points (e.g. convex hulls)
    fn find_two_intersections(&self, other: &Line2<T>) -> (Option<T>, Option<T>) {
        let mut intersections: Vec<T> = Vec::new();

        for edge in self.window_iter() {
            let intersection = edge.intersection_point(*other);
            //println!(
            //    "find_two_intersections found: {:?} for {:?} vs {:?}",
            //    intersection, other, edge
            //);
            match intersection {
                Some(Intersection::Intersection(int_pt)) => {
                    if !intersections.contains(&int_pt) {
                        intersections.push(int_pt);
                    }
                }
                Some(Intersection::Overlap(overlap_line)) => {
                    if !intersections.contains(&overlap_line.start) {
                        intersections.push(overlap_line.start);
                    }
                    if intersections.len() < 2 && !intersections.contains(&overlap_line.end) {
                        intersections.push(overlap_line.end);
                    }
                }
                None => {}
            }

            // If we already found two unique intersections, break
            if intersections.len() == 2 {
                break;
            }
        }

        let first = intersections.first().cloned();
        let second = intersections.get(1).cloned();

        (first, second)
    }

    /// intersection method that returns all possible intersection points
    fn find_all_intersections(&self, other: &Line2<T>) -> Vec<Intersection<T>> {
        let mut intersections: Vec<Intersection<T>> = Vec::new();

        for edge in self.window_iter() {
            if let Some(intersection) = edge.intersection_point(*other) {
                intersections.push(intersection);
            }
        }
        intersections
    }

    /// tests if a ray intersect an edge from p0 to p1
    /// It returns the smallest `t` it can find
    fn ray_edge_intersection(origin: T, ray_dir: T, p0: T, p1: T) -> Option<T::Scalar> {
        let s = p1.sub(p0);
        let r_cross_s = ray_dir.perp_dot(s);

        // If r × s = 0 then the two lines are parallel
        if ulps_eq!(r_cross_s, T::Scalar::zero()) {
            let q_minus_p = p0.sub(origin);
            let q_minus_p_cross_r = q_minus_p.perp_dot(ray_dir);
            ulps_eq!(q_minus_p_cross_r, T::Scalar::zero()).then(|| {
                let r_dot_r = ray_dir.dot(ray_dir);
                let t0 = q_minus_p.dot(ray_dir) / r_dot_r;
                let t1: T::Scalar = t0 + s.dot(ray_dir) / r_dot_r;
                t0.min(t1)
            })
        } else {
            let q_minus_p = p0.sub(origin);
            let t = q_minus_p.perp_dot(s) / r_cross_s;
            let u = q_minus_p.perp_dot(ray_dir) / r_cross_s;

            (t >= T::Scalar::ZERO && (T::Scalar::ZERO..=T::Scalar::one()).contains(&u)).then_some(t)
        }
    }

    /// This method returns the closest ray intersections with the Linestring.
    /// This might return the origin position if the position is already on a line segment
    fn closest_ray_intersection(&self, ray_dir: T, origin: T) -> Option<T> {
        let mut nearest_t: Option<T::Scalar> = None;
        if self.len() <= 1 {
            return None;
        }
        // iterate over every vertex pair as an edge
        self.windows(2).for_each(|v| {
            match (
                Self::ray_edge_intersection(origin, ray_dir, v[0], v[1]),
                nearest_t,
            ) {
                (Some(t), Some(nt)) if t > T::Scalar::ZERO && t < nt => nearest_t = Some(t),
                (Some(t), None) if t > T::Scalar::ZERO => nearest_t = Some(t),
                _ => (),
            }
        });
        nearest_t.map(|t| origin + ray_dir * t)
    }

    /// returns true if this and the other objects are the same.
    /// float equality is determined by ulps_eq!()
    fn approx_eq(&self, other: &Self) -> bool {
        if self.len() != other.len() {
            return false;
        }

        for (point_a, point_b) in self.iter().zip(other.iter()) {
            if !ulps_eq!(point_a.x(), point_b.x()) || !ulps_eq!(point_a.y(), point_b.y()) {
                return false;
            }
        }
        true
    }

    /// Returns an iterator over consecutive pairs of points in the line string, forming continuous lines.
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

    /// Copy this Linestring2 into a Linestring3, populating the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    fn copy_to_3d(&self, plane: Plane) -> Vec<T::Vector3> {
        match plane {
            Plane::XY => self.iter().map(|p2d| p2d.to_3d(T::Scalar::ZERO)).collect(),
            Plane::XZ => self
                .iter()
                .map(|p2d| T::Vector3::new_3d(p2d.x(), T::Scalar::ZERO, p2d.y()))
                .collect(),
            Plane::YZ => self
                .iter()
                .map(|p2d| T::Vector3::new_3d(T::Scalar::ZERO, p2d.y(), p2d.x()))
                .collect(),
        }
    }

    /// Generates the convex hull of the points contained within
    fn convex_hull(&self) -> Result<Vec<T>, LinestringError> {
        convex_hull::graham_scan(self)
    }

    /// Generates the convex hull of the points contained within using a multi-threaded method.
    /// `per_thread_chunk_size` is the number of vertices in each multi-threaded chunk
    fn convex_hull_par(&self, per_thread_chunk_size: usize) -> Result<Vec<T>, LinestringError> {
        assert!(per_thread_chunk_size > 1);
        let indices: Vec<usize> = (0..self.len()).collect();
        Ok(
            convex_hull::convex_hull_par(self, &indices, per_thread_chunk_size)?
                .iter()
                .map(|i| self[*i])
                .collect(),
        )
    }

    /// Simplify a polyline using the Ramer–Douglas–Peucker algorithm.
    ///
    /// The Ramer–Douglas–Peucker algorithm simplifies a polyline by recursively removing points
    /// that are within a specified distance of the line segment connecting the start and end points.
    /// This can be useful for reducing the number of points in a polyline while preserving its shape.
    ///
    /// # Parameters
    ///
    /// - `distance_predicate`: The squared distance threshold. Points within this squared distance
    ///   of the line segment will be removed during simplification.
    ///
    /// # Returns
    ///
    /// A simplified polyline represented as a `Vec<T>`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use vector_traits::glam;
    /// use linestring::prelude::LineString2;
    /// use glam::Vec2;
    ///
    /// let line: Vec<Vec2> = vec![
    ///     [77.0, 613.0].into(),
    ///     [689.0, 650.0].into(),
    ///     [710.0, 467.0].into(),
    ///     [220.0, 200.0].into(),
    ///     [120.0, 300.0].into(),
    ///     [100.0, 100.0].into(),
    ///     [77.0, 613.0].into(),
    /// ];
    ///
    /// // Simplify the polyline using the Ramer–Douglas–Peucker algorithm
    /// let simplified_line = line.simplify_rdp(1.0);
    ///
    /// // The result will have fewer points while preserving the overall shape
    /// assert_eq!(6, simplified_line.windows(2).len());
    /// ```
    ///
    /// In this example, the original polyline `line` is simplified using the Ramer–Douglas–Peucker
    /// algorithm with a squared distance threshold of 1.0. The resulting simplified polyline is then
    /// checked for the expected number of points using `windows(2)`.
    ///
    /// For more information on the Ramer–Douglas–Peucker algorithm, see:
    /// [Ramer–Douglas–Peucker algorithm](https://en.wikipedia.org/wiki/Ramer–Douglas–Peucker_algorithm)
    fn simplify_rdp(&self, distance_predicate: T::Scalar) -> Self {
        if self.len() <= 2 {
            return self.clone();
        }

        let mut rv: Vec<T> = Vec::with_capacity(self.len());
        rv.push(self[0]);
        simplify::simplify_rdp_recursive(&mut rv, self, distance_predicate * distance_predicate);
        rv
    }

    /// Simplify a polyline using the Visvalingam–Whyatt algorithm.
    ///
    /// The Visvalingam–Whyatt algorithm simplifies a polyline by iteratively removing a specified
    /// number of points with the smallest area defined by one point and its neighbors. This can be
    /// useful for reducing the number of points in a polyline while preserving its overall shape.
    ///
    /// # Parameters
    ///
    /// - `points_to_delete`: The number of points to delete during simplification. This parameter
    ///   controls the aggressiveness of the simplification process.
    ///
    /// # Returns
    ///
    /// A simplified polyline represented as a `Vec<T>`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use vector_traits::glam;
    /// use linestring::prelude::LineString2;
    /// use glam::Vec2;
    ///
    ///
    /// let line: Vec<Vec2> = vec![
    ///     [77.0, 613.0].into(),
    ///     [689.0, 650.0].into(),
    ///     [710.0, 467.0].into(),
    ///     [220.0, 200.0].into(),
    ///     [120.0, 300.0].into(),
    ///     [100.0, 100.0].into(),
    ///     [77.0, 613.0].into(),
    /// ];
    ///
    /// // Simplify the polyline using the Visvalingam–Whyatt algorithm, deleting 4 points
    /// let simplified_line = line.simplify_vw(4);
    ///
    /// // The result will have fewer points while preserving the overall shape
    /// assert_eq!(line.len() - 4, simplified_line.len());
    /// ```
    ///
    /// In this example, the original polyline `line` is simplified using the Visvalingam–Whyatt
    /// algorithm with the deletion of 4 points. The resulting simplified polyline is then checked
    /// for the expected number of points using `len()`.
    ///
    /// For more information on the Visvalingam–Whyatt algorithm, see:
    /// [Visvalingam–Whyatt algorithm](https://en.wikipedia.org/wiki/Visvalingam–Whyatt_algorithm)
    #[inline(always)]
    fn simplify_vw(&self, points_to_delete: usize) -> Self {
        simplify::simplify_vw(self, points_to_delete)
    }

    /// Returns true if self is a convex hull and it contains the 'b' point (inclusive)
    /// This function is single threaded.
    /// Will return unpredictable results if self is not a convex hull.
    #[inline(always)]
    fn contains_point_inclusive(&self, p: T) -> bool {
        self.is_connected() && convex_hull::contains_point_inclusive(self, p)
    }

    /// Returns true if self is a convex hull and it contains the 'b' point (exclusive)
    /// This function is single threaded.
    /// Will return unpredictable results if self is not a convex hull.
    #[inline(always)]
    fn contains_point_exclusive(&self, p: T) -> bool {
        self.is_connected() && convex_hull::contains_point_exclusive(self, p)
    }

    /// Apply an operation over each coordinate.
    /// Useful when you want to round the values of the coordinates.
    fn apply<F>(&mut self, f: &F)
    where
        F: Fn(T) -> T,
    {
        for v in self.iter_mut() {
            *v = f(*v)
        }
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
    /// # use vector_traits::glam::Vec2;
    /// use linestring::prelude::LineString2;
    ///
    /// let points = vec![
    ///     Vec2::new(0.5, 0.0),
    ///     Vec2::new(1.0, 0.0),
    ///     Vec2::new(2.0, 0.0),
    ///     Vec2::new(3.0, 0.0),
    /// ];
    ///
    /// let distance = 0.8;
    /// let iterator = points.discretize(distance);
    ///
    /// let result: Vec<Vec2> = iterator.collect();
    ///
    /// // Ensure that the result contains the original points and intermediate steps
    /// assert_eq!(result, vec![
    ///     Vec2::new(0.5, 0.0),
    ///     Vec2::new(1.0, 0.0),
    ///     Vec2::new(1.5, 0.0),
    ///     Vec2::new(2.0, 0.0),
    ///     Vec2::new(2.5, 0.0),
    ///     Vec2::new(3.0, 0.0),
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

/// A workaround for Rust's limitations where external traits cannot be implemented for external types.
///
/// The `Approx` trait provides methods for performing approximate equality comparisons on types.
/// It serves as a workaround for Rust's limitations, allowing you to implement approximate
/// equality checks for types not originally designed with this capability.
///
/// This trait leverages the `approx` crate and its traits to perform approximate equality
/// comparisons. The methods in this trait are wrappers around the corresponding methods provided
/// by the `approx` crate.
///
pub trait Approx<T: GenericVector2> {
    /// Checks if two instances are nearly equal within a specified tolerance in ULPs (Units in the Last Place).
    ///
    /// This method delegates to the `approx::UlpsEq::ulps_eq` method, performing approximate equality checks
    /// one time per coordinate axis.
    fn is_ulps_eq(
        &self,
        other: &Self,
        epsilon: <T::Scalar as AbsDiffEq>::Epsilon,
        max_ulps: u32,
    ) -> bool;

    /// Checks if two instances are nearly equal within a specified absolute difference tolerance.
    ///
    /// This method delegates to the `approx::AbsDiffEq::abs_diff_eq` method, performing approximate equality checks
    /// one time per coordinate axis.
    fn is_abs_diff_eq(&self, other: &Self, epsilon: <T::Scalar as AbsDiffEq>::Epsilon) -> bool;
}

impl<T: GenericVector2> Approx<T> for Vec<T> {
    fn is_ulps_eq(
        &self,
        other: &Self,
        epsilon: <T::Scalar as AbsDiffEq>::Epsilon,
        max_ulps: u32,
    ) -> bool {
        self.len() == other.len()
            && self
                .iter()
                .zip(other.iter())
                .all(|(a, b)| a.is_ulps_eq(*b, epsilon, max_ulps))
    }

    fn is_abs_diff_eq(&self, other: &Self, epsilon: <T::Scalar as AbsDiffEq>::Epsilon) -> bool {
        self.len() == other.len()
            && self
                .iter()
                .zip(other.iter())
                .all(|(a, b)| a.is_abs_diff_eq(*b, epsilon))
    }
}

/// A simple 2d AABB
/// If min_max is none no data has not been assigned yet.
#[derive(Copy, Clone, Debug)]
pub struct Aabb2<T: HasXY> {
    min_max: Option<(T, T)>,
}

impl<T: HasXY> Aabb2<T> {
    /// Creates a degenerate AABB with both minimum and maximum corners set to the provided point.
    pub fn new(point: T) -> Self {
        Self {
            min_max: Some((point, point)),
        }
    }

    /// updates the aabb with points
    pub fn with_points(points: &[T]) -> Self {
        let mut rv = Self { min_max: None };
        if !points.is_empty() {
            rv.update_with_points(points)
        }
        rv
    }

    /// updates the aabb with the limits of another aabb
    pub fn update_aabb(&mut self, aabb: Aabb2<T>) {
        if let Some((min, max)) = aabb.min_max {
            self.update_with_point(min);
            self.update_with_point(max);
        }
    }

    /// update the aabb with a single point
    pub fn update_with_point(&mut self, point: T) {
        if self.min_max.is_none() {
            self.min_max = Some((point, point));
            return;
        }
        let (mut aabb_min, mut aabb_max) = self.min_max.unwrap();
        aabb_min.set_x(aabb_min.x().min(point.x()));
        aabb_min.set_y(aabb_min.y().min(point.y()));
        aabb_max.set_x(aabb_max.x().max(point.x()));
        aabb_max.set_y(aabb_max.y().max(point.y()));
        self.min_max = Some((aabb_min, aabb_max));
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
            aabb_max.set_x(aabb_max.x().max(point.x()));
            aabb_max.set_y(aabb_max.y().max(point.y()));
        }
        self.min_max = Some((aabb_min, aabb_max));
    }

    /// returns the upper limit of the AABB if it exists
    pub fn high(&self) -> Option<T> {
        if let Some((_, high)) = self.min_max {
            return Some(high);
        }
        None
    }

    /// returns the lower limit of the AABB if it exists
    pub fn low(&self) -> Option<T> {
        if let Some((low, _)) = self.min_max {
            return Some(low);
        }
        None
    }

    /// Returns the extents of this AABB in the form of `Option<(min_coordinate, max_coordinate, width, height)>`.
    pub fn extents(&self) -> Option<(T, T, T::Scalar, T::Scalar)> {
        if let Some((low, high)) = self.min_max {
            let width = high.x() - low.x();
            let height = high.y() - low.y();
            Some((low, high, width, height))
        } else {
            None
        }
    }

    /// Returns the extents of this AABB in the form of `Option<(min_coordinate, max_coordinate)>`.
    pub fn min_max(&self) -> Option<(T, T)> {
        self.min_max
    }

    /// returns the center point of the AABB if it exists
    pub fn center(&self) -> Option<T> {
        if let Some((low, high)) = self.min_max {
            return Some(T::new_2d(
                (low.x() + high.x()) / T::Scalar::TWO,
                (low.y() + high.y()) / T::Scalar::TWO,
            ));
        }
        None
    }

    /// returns true if this aabb entirely contains/engulfs 'other' (inclusive)
    #[inline(always)]
    pub fn contains_aabb(&self, other: Aabb2<T>) -> bool {
        if let Some(self_aabb) = self.min_max {
            if let Some(other_aabb) = other.min_max {
                return Self::contains_point_inclusive_(self_aabb, other_aabb.0)
                    && Self::contains_point_inclusive_(self_aabb, other_aabb.1);
            }
        }
        false
    }

    /// returns true if this aabb entirely contains/engulfs a line (inclusive)
    #[inline(always)]
    pub fn contains_line_inclusive(&self, line: &Line2<T>) -> bool {
        if let Some(self_aabb) = self.min_max {
            return Self::contains_point_inclusive_(self_aabb, line.start)
                && Self::contains_point_inclusive_(self_aabb, line.end);
        }
        false
    }

    /// returns true if this aabb contains a point (inclusive)
    #[inline(always)]
    pub fn contains_point_inclusive(&self, point: T) -> bool {
        if let Some(self_aabb) = self.min_max {
            return Self::contains_point_inclusive_(self_aabb, point);
        }
        false
    }

    /// Returns true if the AABB inclusively contains the given point.
    #[inline(always)]
    fn contains_point_inclusive_(aabb: (T, T), point: T) -> bool {
        (aabb.0.x() < point.x() || ulps_eq!(&aabb.0.x(), &point.x()))
            && (aabb.0.y() < point.y() || ulps_eq!(&aabb.0.y(), &point.y()))
            && (aabb.1.x() > point.x() || ulps_eq!(&aabb.1.x(), &point.x()))
            && (aabb.1.y() > point.y() || ulps_eq!(&aabb.1.y(), &point.y()))
    }

    /// Returns the AABB represented as a CCW convex hull, starting from the `min` position
    /// of the AABB. For degenerate AABBs created from a single point, the convex hull
    /// will essentially be a looped point.
    #[inline(always)]
    pub fn convex_hull<R: HasXY>(&self) -> Option<Vec<R>>
    where
        R: HasXY<Scalar = T::Scalar> + HasXY<Scalar = T::Scalar>,
    {
        if let Some((min, max)) = self.min_max {
            Some(vec![
                (R::new_2d(min.x(), min.y())),
                (R::new_2d(max.x(), min.y())),
                (R::new_2d(max.x(), max.y())),
                (R::new_2d(min.x(), max.y())),
                (R::new_2d(min.x(), min.y())),
            ])
        } else {
            None
        }
    }

    /// Apply an operation over each coordinate.
    pub fn apply<F>(&mut self, f: &F)
    where
        F: Fn(T) -> T,
    {
        if let Some(ref mut min_max) = self.min_max {
            self.min_max = Some((f(min_max.0), f(min_max.1)))
        }
    }
}

/// Get any intersection point between line segment and point.
/// Inspired by <https://stackoverflow.com/a/17590923>
pub fn intersect_line_point<T: GenericVector2>(
    line: Line2<T>,
    point: T,
) -> Option<Intersection<T>> {
    // take care of end point equality
    if ulps_eq!(line.start.x(), point.x()) && ulps_eq!(line.start.y(), point.y()) {
        return Some(Intersection::Intersection(point));
    }
    if ulps_eq!(line.end.x(), point.x()) && ulps_eq!(line.end.y(), point.y()) {
        return Some(Intersection::Intersection(point));
    }

    let l0x = line.start.x();
    let l1x = line.end.x();
    let l0y = line.start.y();
    let l1y = line.end.y();
    let px = point.x();
    let py = point.y();

    let ab = ((l1x - l0x) * (l1x - l0x) + (l1y - l0y) * (l1y - l0y)).sqrt();
    let ap = ((px - l0x) * (px - l0x) + (py - l0y) * (py - l0y)).sqrt();
    let pb = ((l1x - px) * (l1x - px) + (l1y - py) * (l1y - py)).sqrt();

    #[cfg(feature = "console_trace")]
    println!("ab={:?}, ap={:?}, pb={:?}, ap+pb={:?}", ab, ap, pb, ap + pb);
    if ulps_eq!(&ab, &(ap + pb)) {
        return Some(Intersection::Intersection(point));
    }
    None
}

pub enum Intersection<T> {
    // Normal one point intersection
    Intersection(T),
    // Collinear overlapping
    Overlap(Line2<T>),
}

impl<T: GenericVector2> Intersection<T> {
    /// return a single, simple intersection point
    pub fn single(&self) -> T {
        match self {
            Self::Overlap(a) => a.start,
            Self::Intersection(a) => *a,
        }
    }

    /// return a the closest of the intersection points or the only one found.
    pub fn closest(&self, other: T) -> T {
        match self {
            Self::Overlap(a) => {
                if other.distance_sq(a.start) < other.distance_sq(a.end) {
                    a.start
                } else {
                    a.end
                }
            }
            Self::Intersection(a) => *a,
        }
    }
}

#[inline(always)]
pub fn scale_to_coordinate<T: GenericVector2>(point: T, vector: T, scale: T::Scalar) -> T {
    point + vector * scale
}

/// The distance between the line a->b to the point p is the same as
/// distance = |(a-p)×(a-b)|/|a-b|
/// <https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Another_vector_formulation>
/// Make sure to *not* call this function with a-b==0
/// This function returns the distance²
#[inline(always)]
pub fn distance_to_line_squared<T: GenericVector2>(l0: T, l1: T, p: T) -> T::Scalar {
    let l0_sub_l1 = l0 - l1;
    let l0_sub_l1_sq_mag = l0_sub_l1.magnitude_sq();
    let l0_sub_p = l0 - p;

    // dot product normalized by the magnitude squared of l0_sub_l1
    let dot = l0_sub_p.dot(l0_sub_l1) / l0_sub_l1_sq_mag;

    if dot < T::Scalar::ZERO {
        l0_sub_p.magnitude_sq()
    } else if dot > T::Scalar::ONE {
        (l1 - p).magnitude_sq()
    } else {
        let a_sub_p_cross_a_sub_b = l0_sub_p.perp_dot(l0_sub_l1);
        a_sub_p_cross_a_sub_b * a_sub_p_cross_a_sub_b / l0_sub_l1_sq_mag
    }
}

/// Same as `distance_to_line_squared<T>` but it can be called when a-b might be 0.
/// It's a little slower because it does the a==b test
#[inline(always)]
pub fn distance_to_line_squared_safe<T: GenericVector2>(l0: T, l1: T, p: T) -> T::Scalar {
    if l0.is_ulps_eq(
        l1,
        T::Scalar::default_epsilon(),
        T::Scalar::default_max_ulps(),
    ) {
        // give the point-to-point answer if the segment is a point
        l0.distance_sq(p)
    } else {
        distance_to_line_squared(l0, l1, p)
    }
}

/// This is a simple but efficient affine transformation object.
/// It can pan, zoom and flip points around center axis but not rotate.
/// It does not handle vector transformation, only points.
#[derive(PartialEq, Clone, Debug)]
pub struct SimpleAffine<T: GenericVector2> {
    /// The offsets used to center the 'source' coordinate system. Typically the input geometry
    /// in this case.
    pub a_offset: [T::Scalar; 2],
    /// A zoom scale
    pub scale: [T::Scalar; 2],
    /// The offsets needed to center coordinates of interest on the 'dest' coordinate system.
    /// i.e. the screen coordinate system.
    pub b_offset: [T::Scalar; 2],
}

impl<T: GenericVector2> SimpleAffine<T> {
    pub fn new(a_aabb: &Aabb2<T>, b_aabb: &Aabb2<T>) -> Result<Self, LinestringError> {
        let min_dim = T::Scalar::ONE;
        let two = T::Scalar::TWO;

        if let Some(source_low) = a_aabb.low() {
            if let Some(source_high) = a_aabb.high() {
                if let Some(destination_low) = b_aabb.low() {
                    if let Some(destination_high) = b_aabb.high() {
                        let source_aabb_center = [
                            -(source_low[0] + source_high[0]) / two,
                            -(source_low[1] + source_high[1]) / two,
                        ];
                        let source_aabb_size = [
                            (source_high[0] - source_low[0]).max(min_dim),
                            (source_high[1] - source_low[1]).max(min_dim),
                        ];

                        let dest_aabb_center = [
                            (destination_low[0] + destination_high[0]) / two,
                            (destination_low[1] + destination_high[1]) / two,
                        ];
                        let dest_aabb_size = [
                            (destination_high[0] - destination_low[0]).max(min_dim),
                            (destination_high[1] - destination_low[1]).max(min_dim),
                        ];

                        // make sure the larges dimension of source fits inside smallest of dest
                        let source_aabb_size = source_aabb_size[0].max(source_aabb_size[1]);
                        let dest_aabb_size = dest_aabb_size[0].min(dest_aabb_size[1]);
                        let scale = dest_aabb_size / source_aabb_size;
                        return Ok(Self {
                            a_offset: source_aabb_center,
                            scale: [scale, scale],
                            b_offset: dest_aabb_center,
                        });
                    }
                }
            }
        }
        Err(LinestringError::AabbError(
            "could not get dimension of the AABB".to_string(),
        ))
    }

    /// transform from dest (b) coordinate system to source (a) coordinate system

    #[inline(always)]
    pub fn transform_ba(&self, point: T) -> Result<T, LinestringError> {
        let x = (point.x() - self.b_offset[0]) / self.scale[0] - self.a_offset[0];
        let y = (point.y() - self.b_offset[1]) / self.scale[1] - self.a_offset[1];
        if x.is_finite() && y.is_finite() {
            Ok(T::new_2d(x, y))
        } else {
            Err(LinestringError::TransformError(
                "Transformation out of bounds".to_string(),
            ))
        }
    }

    /// Transform from source (a) coordinate system to dest (b) coordinate system

    #[inline(always)]
    pub fn transform_ab(&self, point: T) -> Result<T, LinestringError> {
        let x = (point.x() + self.a_offset[0]) * self.scale[0] + self.b_offset[0];
        let y = (point.y() + self.a_offset[1]) * self.scale[1] + self.b_offset[1];
        if x.is_finite() && y.is_finite() {
            Ok(T::new_2d(x, y))
        } else {
            Err(LinestringError::TransformError(
                "Transformation out of bounds".to_string(),
            ))
        }
    }

    /// transform an array from dest (b) coordinate system to source (a) coordinate system
    #[inline(always)]
    pub fn transform_ba_a(
        &self,
        points: [T::Scalar; 4],
    ) -> Result<[T::Scalar; 4], LinestringError> {
        let x1 = (points[0] - self.b_offset[0]) / self.scale[0] - self.a_offset[0];
        let y1 = (points[1] - self.b_offset[1]) / self.scale[1] - self.a_offset[1];
        let x2 = (points[2] - self.b_offset[0]) / self.scale[0] - self.a_offset[0];
        let y2 = (points[3] - self.b_offset[1]) / self.scale[1] - self.a_offset[1];
        if x1.is_finite() && y1.is_finite() && x2.is_finite() && y2.is_finite() {
            Ok([x1, y1, x2, y2])
        } else {
            Err(LinestringError::TransformError(
                "Transformation out of bounds".to_string(),
            ))
        }
    }

    /// transform an array from source (a) coordinate system to dest (b) coordinate system
    #[inline(always)]
    pub fn transform_ab_a(
        &self,
        points: [T::Scalar; 4],
    ) -> Result<[T::Scalar; 4], LinestringError> {
        let x1 = (points[0] + self.a_offset[0]) * self.scale[0] + self.b_offset[0];
        let y1 = (points[1] + self.a_offset[1]) * self.scale[1] + self.b_offset[1];
        let x2 = (points[2] + self.a_offset[0]) * self.scale[0] + self.b_offset[0];
        let y2 = (points[3] + self.a_offset[1]) * self.scale[1] + self.b_offset[1];

        if x1.is_finite() && y1.is_finite() && x2.is_finite() && y2.is_finite() {
            Ok([x1, y1, x2, y2])
        } else {
            Err(LinestringError::TransformError(
                "Transformation out of bounds".to_string(),
            ))
        }
    }
}

pub struct DiscreteLineSegmentIterator<'a, T: GenericVector2> {
    points: &'a Vec<T>,
    distance: T::Scalar,
    current_index: usize,
    substep_index: usize,
    num_substeps: usize,
    step: T,
}

impl<'a, T: GenericVector2> DiscreteLineSegmentIterator<'a, T> {
    fn new(points: &'a Vec<T>, distance: T::Scalar) -> Self {
        DiscreteLineSegmentIterator {
            points,
            distance,
            current_index: 0,
            substep_index: 0,
            num_substeps: 0,
            step: T::new_2d(T::Scalar::ZERO, T::Scalar::ZERO),
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
            self.step = T::new_2d(T::Scalar::ZERO, T::Scalar::ZERO);
        }
    }
}

impl<'a, T: GenericVector2> Iterator for DiscreteLineSegmentIterator<'a, T>
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
