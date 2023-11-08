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

use crate::{
    linestring_3d,
    linestring_3d::{LineString3, Plane},
    LinestringError,
};
use collections::binary_heap::BinaryHeap;
use itertools::Itertools;
use std::{collections, fmt::Debug};
use vector_traits::{
    approx::*,
    num_traits::{real::Real, One, Zero},
    GenericScalar, GenericVector2, HasXY, HasXYZ,
};

/// Module containing the convex hull calculations
pub mod convex_hull;
pub mod impls;
/// Module containing the intersection calculations but for indexed vertexes
pub mod indexed_intersection;
/// Module containing the intersection calculations
pub mod intersection;

/// A 2D polyline representation with rich geometric operations.
///
/// `LineString2` provides a zero-cost abstraction over a series of 2D points. When the last
/// and the first point of the line are identical, the line is considered 'connected' or closed.
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
/// - **Convex Hulls** (When "connected"):
///   - Generate convex hulls using both Gift Wrapping and Graham's Scan algorithms.
///   - Inclusion tests for points, AABBs, and other convex hulls, both in parallel and single-threaded versions.
///
/// In its essence, this structure provides foundational utilities for handling and manipulating
/// polylines in a 2D space.
///
#[derive(Clone)]
pub struct LineString2<T: GenericVector2>(pub Vec<T>);

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
    pub fn discretize_2d(&self, max_dist: T::Scalar) -> LineString2<T> {
        let mut rv = LineString2::with_point(self.start_point);

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
                rv.0.push(p);
                cur_x = new_x;
                cur_y = new_y;
            } else {
                point_stack.push(mid_x);
            }
        }

        // Update last point.
        let last_position = rv.0.len() - 1;
        rv.0[last_position] = self.end_point;
        rv
    }

    /// Convert this parable abstraction into a single straight line
    pub fn discretize_3d_straight_line(&self) -> LineString3<T::Vector3> {
        let mut rv = LineString3::default().with_connected(false);
        let distance =
            -distance_to_line_squared_safe(self.segment.start, self.segment.end, self.start_point)
                .sqrt();
        rv.points.push(T::Vector3::new_3d(
            self.start_point.x(),
            self.start_point.y(),
            distance,
        ));
        let distance = -self.end_point.distance(self.cell_point);
        rv.points.push(T::Vector3::new_3d(
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
    pub fn discretize_3d(&self, max_dist: T::Scalar) -> LineString3<T::Vector3> {
        let mut rv = LineString3::default().with_connected(false);
        let z_comp = -self.start_point.distance(self.cell_point);
        rv.points.push(T::Vector3::new_3d(
            self.start_point.x(),
            self.start_point.y(),
            z_comp,
        ));

        let z_comp = -self.end_point.distance(self.cell_point);
        // todo, don't insert end_point and then pop it again a few lines later..
        rv.points.push(T::Vector3::new_3d(
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
            sqr_segment_length * Self::point_projection_3d(&rv.points[0], &self.segment);
        let projection_end =
            sqr_segment_length * Self::point_projection_3d(&rv.points[1], &self.segment);

        // Compute parabola parameters in the transformed space.
        // Parabola has next representation:
        // f(x) = ((x-rot_x)^2 + rot_y^2) / (2.0*rot_y).
        let point_vec_x = self.cell_point.x() - self.segment.start.x();
        let point_vec_y = self.cell_point.y() - self.segment.start.y();
        let rot_x = segm_vec_x * point_vec_x + segm_vec_y * point_vec_y;
        let rot_y = segm_vec_x * point_vec_y - segm_vec_y * point_vec_x;

        // Save the last point.
        let last_point = (*rv.points)[1];
        let _ = rv.points.pop();

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
                rv.points.push(T::Vector3::new_3d(inter_x, inter_y, z_comp));
                cur_x = new_x;
                cur_y = new_y;
            } else {
                point_stack.push(mid_x);
            }
        }

        // Update last point.
        let last_position = rv.points.len() - 1;
        rv.points[last_position] = last_point;
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

pub struct LineIterator<'a, T: GenericVector2>(std::slice::Windows<'a, T>);

impl<'a, T: GenericVector2> LineIterator<'a, T> {
    fn len(&self) -> usize {
        self.0.len()
    }
}

impl<T: GenericVector2> LineString2<T> {
    /// creates a new linestring with enough capacity allocated to store `capacity` points.
    /// If you want to store a square of points in a loop, you will need to use the `capacity`=5
    pub fn with_capacity(capacity: usize) -> Self {
        Self(Vec::<T>::with_capacity(capacity))
    }

    /// Returns `true` if the last and first points of the collection are exactly the same,
    /// indicating a closed loop.
    /// Note that an empty linestring is also "closed" since first() and last() object are the
    /// same. I.e. None
    pub fn is_connected(&self) -> bool {
        self.0.is_empty() || self.0.first().unwrap() == self.0.last().unwrap()
    }

    /// Copies the points of the iterator into the LineString2
    /// from_iter is already claimed for into() objects.
    pub fn with_iter_ref<'b, I>(iter: I) -> Self
    where
        T: 'b,
        I: Iterator<Item = &'b T>,
    {
        Self(iter.cloned().collect())
    }

    pub fn with_iter<I>(iter: I) -> Self
    where
        I: Iterator<Item = T>,
    {
        Self(iter.collect())
    }

    pub fn with_vec(v: Vec<T>) -> Self {
        Self(v)
    }

    pub fn with_point(v: T) -> Self {
        Self(vec![v])
    }

    /// Moves all the elements of `other` into `Self`, leaving `other` empty.
    /// TODO: currently ignores if `other` is connected or not.
    /// # Panics
    /// Panics if the number of elements in the points vector overflows a `usize`.
    pub fn append(&mut self, mut other: Self) {
        self.0.append(&mut other.0);
    }

    /// Returns true if the lines are self intersecting
    /// If number of points < 10 then the intersections are tested using brute force O(n²)
    /// If more than that a sweep-line algorithm is used O(n*log(n)+i*log(n))
    pub fn is_self_intersecting(&self) -> Result<bool, LinestringError> {
        if self.0.len() <= 2 {
            Ok(false)
        } else if self.0.len() < 10 {
            //let lines = self.as_lines_iter();
            let iter = self.iter();
            let iter_len = iter.len();
            for l0 in iter.enumerate().take(iter_len - 1) {
                for l1 in self.iter().enumerate().skip(l0.0 + 1) {
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
                            /*println!(
                                "intersection at {:?} {}:{:?}, {}:{:?}",
                                point, l0.0, l0.1, l1.0, l1.1
                            );*/
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
                .with_lines(self.iter())?
                .compute()?;
            //print!("Lines rv={} [", result.is_empty());
            //for p in self.as_lines().iter() {
            //print!("[{:?},{:?}]-[{:?},{:?}],", p.start.x, p.start.y, p.end.x, p.end.y);
            //    print!("[{:?},{:?},{:?},{:?}],", p.start.x, p.start.y, p.end.x, p.end.y);
            //print!("[{:?},{:?}],", p.x, p.y,);
            //}
            //println!("]");
            Ok(result.len() > 0)
        }
    }

    /// This intersection method returns the first intersection it finds.
    pub fn intersection(&self, other: &Line2<T>) -> Option<Intersection<T>> {
        for l in self.iter() {
            let intersection = l.intersection_point(*other);
            if intersection.is_some() {
                return intersection;
            }
        }
        None
    }

    /// intersection method that returns two possible intersection points
    /// Use this when you know there can only be two intersection points (e.g. convex hulls)
    pub fn find_two_intersections(&self, other: &Line2<T>) -> (Option<T>, Option<T>) {
        let mut intersections: Vec<T> = Vec::new();

        for edge in self.iter() {
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
    pub fn find_all_intersections(&self, other: &Line2<T>) -> Vec<Intersection<T>> {
        let mut intersections: Vec<Intersection<T>> = Vec::new();

        for edge in self.iter() {
            if let Some(intersection) = edge.intersection_point(*other) {
                intersections.push(intersection);
            }
        }
        intersections
    }

    /// tests if a ray intersect an edge from p0 to p1
    /// It returns the smallest `t` it can find
    pub fn ray_edge_intersection(origin: T, ray_dir: T, p0: T, p1: T) -> Option<T::Scalar> {
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
    pub fn closest_ray_intersection(&self, ray_dir: T, origin: T) -> Option<T> {
        let mut nearest_t: Option<T::Scalar> = None;
        if self.0.len() <= 1 {
            return None;
        }
        // iterate over every vertex pair as an edge
        self.0.windows(2).for_each(|v| {
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

    #[inline(always)]
    pub fn points(&self) -> &Vec<T> {
        &self.0
    }

    /// returns the number of points in the point list
    /// not the number of segments
    pub fn point_count(&self) -> usize {
        self.0.len()
    }

    /// returns true if the point list is empty
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// returns true if this and the other objects are the same.
    /// float equality is determined by ulps_eq!()
    pub fn approx_eq(&self, other: &Self) -> bool {
        if self.0.len() != other.0.len() {
            return false;
        }

        for (point_a, point_b) in self.0.iter().zip(other.0.iter()) {
            if !ulps_eq!(point_a.x(), point_b.x()) || !ulps_eq!(point_a.y(), point_b.y()) {
                return false;
            }
        }
        true
    }

    /// Returns the line string as a iterator of Line2
    #[must_use = "iterator adaptors are lazy and do nothing unless consumed"]
    pub fn iter(&self) -> LineIterator<'_, T> {
        match self.0.len() {
            0 | 1 => LineIterator([].windows(2)), // Empty iterator
            _ => LineIterator(self.0.windows(2)),
        }
    }

    /// Copy this Linestring2 into a Linestring3, populating the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    pub fn copy_to_3d(&self, plane: Plane) -> LineString3<T::Vector3> {
        match plane {
            Plane::XY => self
                .0
                .iter()
                .map(|p2d| T::Vector3::new_3d(p2d.x(), p2d.y(), T::Scalar::ZERO))
                .collect(),
            Plane::XZ => self
                .0
                .iter()
                .map(|p2d| T::Vector3::new_3d(p2d.x(), T::Scalar::ZERO, p2d.y()))
                .collect(),
            Plane::YZ => self
                .0
                .iter()
                .map(|p2d| T::Vector3::new_3d(T::Scalar::ZERO, p2d.x(), p2d.y()))
                .collect(),
        }
    }

    pub fn push(&mut self, point: T) {
        self.0.push(point);
    }

    /// Simplify using Ramer–Douglas–Peucker algorithm
    pub fn simplify_rdp(&self, distance_predicate: T::Scalar) -> Self {
        if self.0.len() <= 2 {
            return self.clone();
        }

        let mut rv: Vec<T> = Vec::with_capacity(self.0.len());
        rv.push(self.0[0]);
        Self::simplify_rdp_recursive(&mut rv, &self.0, distance_predicate * distance_predicate);
        Self(rv)
    }

    fn simplify_rdp_recursive(rv: &mut Vec<T>, points: &[T], distance_predicate_sq: T::Scalar) {
        if points.len() <= 2 {
            rv.push(*points.last().unwrap());
            return;
        }

        let start_point = points[0];
        let end_point = points[points.len() - 1];

        let (max_dist_sq, index) = points
            .iter()
            .enumerate()
            .skip(1)
            .take(points.len() - 2)
            .map(|(i, &point)| {
                (
                    distance_to_line_squared_safe(start_point, end_point, point),
                    i,
                )
            })
            .max_by(|(dist1, _), (dist2, _)| {
                dist1
                    .partial_cmp(dist2)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap_or((-T::Scalar::ONE, 0)); // default that's unlikely to be used

        if max_dist_sq <= distance_predicate_sq {
            rv.push(end_point);
        } else {
            Self::simplify_rdp_recursive(rv, &points[..index + 1], distance_predicate_sq);
            Self::simplify_rdp_recursive(rv, &points[index..], distance_predicate_sq);
        }
    }

    /// Simplify using Visvalingam–Whyatt algorithm. This algorithm will delete 'points_to_delete'
    /// of points from the polyline with the smallest area defined by one point and it's neighbours.
    pub fn simplify_vw(&self, points_to_delete: usize) -> Self {
        let connected = self.is_connected();
        if self.0.len() <= 2 {
            // Nothing to do here, we can't delete endpoints
            return self.clone();
        }
        // priority queue key: area, value: indices of self.points + a copy of area.
        // When a point is removed it's previously calculated area-to-neighbour value will not be
        // removed. Instead new areas will simply be added to the priority queue.
        // If a removed node is pop():ed it will be checked against the link_tree hash map.
        // If it is not in there or if the area doesn't match it will simply be ignored and a
        // new value pop():ed.

        let mut search_tree = BinaryHeap::<PriorityDistance<T>>::new();
        // map from node number to remaining neighbours of that node. All indices of self.points
        // AHashMap::<node_id:usize, (prev_node_id:usize, next_node_id:usize, area:T)>
        let mut link_tree = ahash::AHashMap::<usize, (usize, usize, T::Scalar)>::default();
        {
            let mut iter_i = self.0.iter().enumerate();
            let mut iter_j = self.0.iter().enumerate().skip(1);
            // the k iterator will terminate before i & j, so the iter_i & iter_j unwrap()s are safe
            for k in self.0.iter().enumerate().skip(2) {
                let i = iter_i.next().unwrap();
                let j = iter_j.next().unwrap();
                // define the area between point i, j & k as search criteria
                let area = Line2::triangle_area_squared_times_4(*i.1, *j.1, *k.1);
                search_tree.push(PriorityDistance {
                    key: area,
                    index: j.0,
                });
                // point j is connected to point i and k
                let _ = link_tree.insert(j.0, (i.0, k.0, area));
            }
        }
        if connected {
            // add an extra point at the end, faking the loop
            let i = self.0.len() - 2;
            let j = self.0.len() - 1;
            let k = self.0.len();
            let area = Line2::triangle_area_squared_times_4(self.0[i], self.0[j], self.0[0]);
            search_tree.push(PriorityDistance {
                key: area,
                index: j,
            });
            let _ = link_tree.insert(j, (i, k, area));
        }

        let self_points_len = self.0.len();

        let mut deleted_nodes: usize = 0;
        loop {
            if search_tree.is_empty() || link_tree.len() == 2 || deleted_nodes >= points_to_delete {
                break;
            }

            if let Some(smallest) = search_tree.pop() {
                if let Some(old_links) = link_tree.get(&smallest.index).copied() {
                    let area = old_links.2;
                    if smallest.key != area {
                        // we hit a lazily deleted node, try again
                        continue;
                    } else {
                        let _ = link_tree.remove(&smallest.index);
                    }
                    deleted_nodes += 1;

                    let prev = old_links.0;
                    let next = old_links.1;

                    let prev_prev: Option<usize> = link_tree.get(&prev).map(|link| link.0);
                    let next_next: Option<usize> = link_tree.get(&next).map(|link| link.1);

                    if let Some(next_next) = next_next {
                        if let Some(prev_prev) = prev_prev {
                            let area = Line2::triangle_area_squared_times_4(
                                self.0[prev],
                                self.0[next % self_points_len],
                                self.0[next_next % self_points_len],
                            );
                            search_tree.push(PriorityDistance {
                                key: area,
                                index: next,
                            });
                            let _ = link_tree.insert(next, (prev, next_next, area));

                            let area = Line2::triangle_area_squared_times_4(
                                self.0[prev_prev],
                                self.0[prev],
                                self.0[next % self_points_len],
                            );
                            search_tree.push(PriorityDistance {
                                key: area,
                                index: prev,
                            });
                            let _ = link_tree.insert(prev, (prev_prev, next, area));
                            continue;
                        }
                    }

                    if let Some(prev_prev) = prev_prev {
                        let area = Line2::triangle_area_squared_times_4(
                            self.0[prev_prev],
                            self.0[prev],
                            self.0[next % self_points_len],
                        );
                        search_tree.push(PriorityDistance {
                            key: area,
                            index: prev,
                        });
                        let _ = link_tree.insert(prev, (prev_prev, next, area));
                        continue;
                    };

                    if let Some(next_next) = next_next {
                        let area = Line2::triangle_area_squared_times_4(
                            self.0[prev],
                            self.0[next % self_points_len],
                            self.0[next_next % self_points_len],
                        );
                        search_tree.push(PriorityDistance {
                            key: area,
                            index: next,
                        });
                        let _ = link_tree.insert(next, (prev, next_next, area));

                        continue;
                    };
                } else {
                    // we hit a lazily deleted node, try again
                    continue;
                }
            }
        }

        // Todo: we *know* the order of the points, remove sorted_unstable()
        // we just don't know the first non-deleted point after start :/
        if !connected {
            [0_usize]
                .iter()
                .copied()
                .chain(
                    link_tree
                        .keys()
                        .sorted_unstable()
                        .copied()
                        .chain([self.0.len() - 1].iter().copied()),
                )
                .map(|x| self.0[x])
                .collect::<Self>()
        } else {
            [0_usize]
                .iter()
                .copied()
                .chain(link_tree.keys().sorted_unstable().copied())
                .map(|x| self.0[x])
                .collect::<Self>()
        }
    }

    /// Returns true if self is a convex hull and it contains the 'b' point (inclusive)
    /// This function is single threaded.
    /// Will return unpredictable results if self is not a convex hull.
    #[inline(always)]
    pub fn contains_point_inclusive(&self, p: T) -> bool {
        self.is_connected() && convex_hull::contains_point_inclusive(self, p)
    }

    /// Returns true if self is a convex hull and it contains the 'b' point (exclusive)
    /// This function is single threaded.
    /// Will return unpredictable results if self is not a convex hull.
    #[inline(always)]
    pub fn contains_point_exclusive(&self, p: T) -> bool {
        self.is_connected() && convex_hull::contains_point_exclusive(self, p)
    }

    /// Apply an operation over each coordinate.
    /// Useful when you want to round the values of the coordinates.
    pub fn apply<F>(&mut self, f: &F)
    where
        F: Fn(T) -> T,
    {
        for v in self.0.iter_mut() {
            *v = f(*v)
        }
    }
}
/*
pub trait Matrix<T:GenericVector2> {
    fn transform(&self, v:T) -> T;
}*/

/// A set of 2d LineString, an aabb + convex_hull.
/// It also contains a list of aabb & convex_hulls of shapes this set has gobbled up.
/// This can be useful for separating out inner regions of the shape.
///
/// This struct is intended to contain related shapes. E.g. outlines of letters with holes
#[derive(Clone)]
pub struct LineStringSet2<T: GenericVector2> {
    set: Vec<LineString2<T>>,
    aabb: Aabb2<T>,
    convex_hull: Option<LineString2<T>>,
    pub internals: Option<Vec<(Aabb2<T>, LineString2<T>)>>,
    //#[doc(hidden)]
    //_pd :PhantomData<M>
}

impl<T: GenericVector2> LineStringSet2<T> {
    /// steal the content of 'other' leaving it empty
    pub fn steal_from(other: &mut LineStringSet2<T>) -> Self {
        //println!("stealing from other.aabb:{:?}", other.aabb);
        let mut set = Vec::<LineString2<T>>::new();
        set.append(&mut other.set);
        Self {
            set,
            aabb: other.aabb,
            convex_hull: other.convex_hull.take(),
            internals: other.internals.take(),
            //_pd:PhantomData,
        }
    }

    pub fn set(&self) -> &Vec<LineString2<T>> {
        &self.set
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            set: Vec::<LineString2<T>>::with_capacity(capacity),
            aabb: Aabb2::default(),
            convex_hull: None,
            internals: None,
            //_pd:PhantomData,
        }
    }

    pub fn get_internals(&self) -> Option<&Vec<(Aabb2<T>, LineString2<T>)>> {
        self.internals.as_ref()
    }

    pub fn is_empty(&self) -> bool {
        self.set.is_empty()
    }

    pub fn push(&mut self, ls: LineString2<T>) {
        if !ls.is_empty() {
            self.set.push(ls);

            for ls in self.set.last().unwrap().0.iter() {
                self.aabb.update_with_point(*ls);
            }
        }
    }

    /// returns the combined convex hull of all the shapes in self.set
    pub fn get_convex_hull(&self) -> &Option<LineString2<T>> {
        &self.convex_hull
    }

    /// calculates the combined convex hull of all the shapes in self.set
    pub fn calculate_convex_hull(&mut self) -> Result<&LineString2<T>, LinestringError> {
        let tmp: Vec<_> = self.set.iter().flat_map(|x| x.points()).cloned().collect();
        self.convex_hull = Some(convex_hull::graham_scan(&tmp)?);
        Ok(self.convex_hull.as_ref().unwrap())
    }

    /// Returns the axis aligned bounding box of this set.
    pub fn get_aabb(&self) -> Aabb2<T> {
        self.aabb
    }

    /*/// Transform each individual component of this set using the transform matrix.
    /// Return the result in a new object.
    pub fn transform(&self, matrix3x3: &M) -> Self {
        let internals = self.internals.as_ref().map(|internals| {
            internals
                .iter()
                .map(|(aabb, line)| (transform_aabb2(matrix3x3, aabb),transform_linestring2( matrix3x3,line)))
                .collect()
        });

        let convex_hull = self
            .convex_hull
            .as_ref()
            .map(|convex_hull| transform_linestring2(matrix3x3, convex_hull));

        Self {
            aabb: transform_aabb2(matrix3x3, &self.aabb),
            set: self.set.iter().map(|x| transform_linestring2(matrix3x3, x)).collect(),
            convex_hull,
            internals,
            _pd:PhantomData,
        }
    }*/

    /// Copy this linestringset2 into a linestringset3, populating the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    /// The empty axis will be set to zero.
    pub fn copy_to_3d(&self, plane: Plane) -> linestring_3d::LineStringSet3<T::Vector3> {
        let mut rv = linestring_3d::LineStringSet3::<T::Vector3>::with_capacity(self.set.len());
        for ls in self.set.iter() {
            rv.push(ls.copy_to_3d(plane));
        }
        rv
    }

    /// drains the 'other' container of all shapes and put them into 'self'
    pub fn take_from(&mut self, mut other: Self) {
        self.aabb.update_aabb(other.aabb);
        self.set.append(&mut other.set);
    }

    /// drains the 'other' container of all shapes and put them into 'self'
    /// The other container must be entirely 'inside' the convex hull of 'self'
    /// The 'other' container must also contain valid 'internals' and 'convex_hull' fields
    pub fn take_from_internal(&mut self, other: &mut Self) -> Result<(), LinestringError> {
        // sanity check
        if other.convex_hull.is_none() {
            return Err(LinestringError::InvalidData(
                "'other' did not contain a valid 'convex_hull' field".to_string(),
            ));
        }
        if self.aabb.low().is_none() {
            //println!("self.aabb {:?}", self.aabb);
            //println!("other.aabb {:?}", other.aabb);
            return Err(LinestringError::InvalidData(
                "'self' did not contain a valid 'aabb' field".to_string(),
            ));
        }
        if other.aabb.low().is_none() {
            return Err(LinestringError::InvalidData(
                "'other' did not contain a valid 'aabb' field".to_string(),
            ));
        }
        if !self.aabb.contains_aabb(other.aabb) {
            //println!("self.aabb {:?}", self.aabb);
            //println!("other.aabb {:?}", other.aabb);
            return Err(LinestringError::InvalidData(
                "The 'other.aabb' is not contained within 'self.aabb'".to_string(),
            ));
        }
        if self.internals.is_none() {
            self.internals = Some(Vec::<(Aabb2<T>, LineString2<T>)>::new())
        }

        self.set.append(&mut other.set);

        if let Some(ref mut other_internals) = other.internals {
            // self.internals.unwrap is safe now
            self.internals.as_mut().unwrap().append(other_internals);
        }

        self.internals
            .as_mut()
            .unwrap()
            .push((other.aabb, other.convex_hull.take().unwrap()));
        Ok(())
    }

    /// Apply an operation over each coordinate in the contained objects.
    /// Useful when you want to round the value of each contained coordinate.
    pub fn apply<F: Fn(T) -> T>(&mut self, f: &F) {
        for s in self.set.iter_mut() {
            s.apply(f);
        }
        self.aabb.apply(f);
        if let Some(ref mut convex_hull) = self.convex_hull {
            convex_hull.apply(f);
        }
        if let Some(ref mut internals) = self.internals {
            for i in internals.iter_mut() {
                i.0.apply(f);
                i.1.apply(f);
            }
        }
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

/// Same as distance_to_line_squared<T> but it can be called when a-b might be 0.
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
