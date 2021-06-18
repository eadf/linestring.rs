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

use super::nalgebra_3d;
#[allow(unused_imports)]
use crate::LinestringError;
use approx::{ulps_eq, AbsDiffEq, RelativeEq, UlpsEq};

use itertools::Itertools;
#[allow(unused_imports)]
#[cfg(feature = "rayon")]
use rayon::prelude::*;
use std::collections;
use std::fmt;

/// Module containing the convex hull calculations
pub mod convex_hull;
/// Module containing the intersection calculations
pub mod intersection;

/// Placeholder for different 2d shapes
pub enum Shape2d<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    Line(Line2<T>),
    Linestring(LineString2<T>),
    ParabolicArc(VoronoiParabolicArc<T>),
}

/// A 2d line
#[derive(PartialEq, Eq, Copy, Clone, Hash, fmt::Debug)]
pub struct Line2<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    pub start: nalgebra::Point2<T>,
    pub end: nalgebra::Point2<T>,
}

impl<T> Line2<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    pub fn new(start: nalgebra::Point2<T>, end: nalgebra::Point2<T>) -> Self {
        Self { start, end }
    }

    /// Get any intersection point between line segments.
    /// Note that this function always detects endpoint-to-endpoint intersections.
    /// Most of this is from <https://stackoverflow.com/a/565282>
    #[allow(clippy::many_single_char_names)]
    #[allow(clippy::suspicious_operation_groupings)]
    pub fn intersection_point(&self, other: &Self) -> Option<Intersection<T>>
    where
        T: nalgebra::RealField
            + Sync
            + AbsDiffEq<Epsilon = T>
            + UlpsEq<Epsilon = T>
            + RelativeEq<Epsilon = T>
            + UlpsEq<Epsilon = T>
            + num_traits::cast::NumCast,
    {
        // AABB tests
        if self.end.x > other.end.x
            && self.end.x > other.start.x
            && self.start.x > other.end.x
            && self.start.x > other.start.x
        {
            return None;
        }
        if self.end.x < other.end.x
            && self.end.x < other.start.x
            && self.start.x < other.end.x
            && self.start.x < other.start.x
        {
            return None;
        }
        if self.end.y > other.end.y
            && self.end.y > other.start.y
            && self.start.y > other.end.y
            && self.start.y > other.start.y
        {
            return None;
        }
        if self.end.y < other.end.y
            && self.end.y < other.start.y
            && self.start.y < other.end.y
            && self.start.y < other.start.y
        {
            return None;
        }

        let p = self.start;
        let q = other.start;
        let r = sub(&self.end, &p);
        let s = sub(&other.end, &q);

        let r_cross_s = cross_z(&r, &s);
        let q_minus_p = sub(&q, &p);
        let q_minus_p_cross_r = cross_z(&q_minus_p, &r);

        // If r × s = 0 then the two lines are parallel
        if ulps_eq!(&r_cross_s, &T::zero()) {
            // one (or both) of the lines may be a point
            let self_is_a_point = point_ulps_eq(&self.start, &self.end);
            let other_is_a_point = point_ulps_eq(&other.start, &other.end);
            if self_is_a_point || other_is_a_point {
                if self_is_a_point && other_is_a_point && point_ulps_eq(&self.start, &other.start) {
                    return Some(Intersection::Intersection(self.start));
                }
                return if self_is_a_point {
                    intersect_line_point(other, &self.start)
                } else {
                    intersect_line_point(self, &other.start)
                };
            }

            // If r × s = 0 and (q − p) × r = 0, then the two lines are collinear.
            if ulps_eq!(&q_minus_p_cross_r, &T::zero()) {
                let r_dot_r = dot(&r, &r);
                let r_div_r_dot_r = div(&r, r_dot_r);
                let s_dot_r = dot(&s, &r);
                let t0 = dot(&q_minus_p, &r_div_r_dot_r);
                let t1 = t0 + s_dot_r / r_dot_r;

                Some(Intersection::OverLap(Line2::new(
                    scale_to_coordinate(&p, &r, t0),
                    scale_to_coordinate(&p, &r, t1),
                )))
            } else {
                // If r × s = 0 and (q − p) × r ≠ 0,
                // then the two lines are parallel and non-intersecting.
                None
            }
        } else {
            // the lines are not parallel
            let t = cross_z(&q_minus_p, &div(&s, r_cross_s));
            let u = cross_z(&q_minus_p, &div(&r, r_cross_s));

            // If r × s ≠ 0 and 0 ≤ t ≤ 1 and 0 ≤ u ≤ 1,
            // the two line segments meet at the point p + t r = q + u s.
            if T::zero() <= t && t <= T::one() && T::zero() <= u && u <= T::one() {
                Some(Intersection::Intersection(scale_to_coordinate(&p, &r, t)))
            } else {
                None
            }
        }
    }

    /// Intersection test for lines known to be connected by a middle point.
    /// This function will *not* report the middle-point as an intersection.
    /// Todo: how to handle start==middle||middle==end
    pub fn intersection_point3(
        start: &nalgebra::Point2<T>,
        middle: &nalgebra::Point2<T>,
        end: &nalgebra::Point2<T>,
    ) -> Result<Option<Intersection<T>>, LinestringError> {
        let middle_sub_start = nalgebra::Vector2::<T>::new(middle.x - start.x, middle.y - start.x);
        let middle_sub_end = nalgebra::Vector2::<T>::new(middle.x - end.x, middle.y - end.y);
        let start_is_vertical = ulps_eq!(&middle_sub_start.x, &T::zero());
        let end_is_vertical = ulps_eq!(&middle_sub_end.x, &T::zero());

        if start_is_vertical && end_is_vertical {
            // both lines are vertical
            if middle_sub_start.y.is_sign_negative() && !middle_sub_end.y.is_sign_negative() {
                // opposite direction
                return Ok(None);
            } else {
                // pick the shortest vector for overlap point
                if (middle_sub_start.x * middle_sub_start.x
                    + middle_sub_start.y * middle_sub_start.y)
                    < (middle_sub_end.x * middle_sub_end.x + middle_sub_end.y * middle_sub_end.y)
                {
                    return Ok(Some(Intersection::OverLap(Line2 {
                        start: *start,
                        end: *middle,
                    })));
                } else {
                    return Ok(Some(Intersection::OverLap(Line2 {
                        start: *middle,
                        end: *end,
                    })));
                }
            }
        } else if start_is_vertical || end_is_vertical {
            return Ok(None);
        }

        // both lines should now be non-vertical, we can compare their slope
        let start_slope = middle_sub_start.x / middle_sub_start.y;
        let end_slope = middle_sub_end.x / middle_sub_end.y;

        if !ulps_eq!(&start_slope, &end_slope) {
            return Ok(None);
        }

        // Slope identical, pick the shortest vector for overlap point
        if (middle_sub_start.x * middle_sub_start.x + middle_sub_start.y * middle_sub_start.y)
            < (middle_sub_end.x * middle_sub_end.x + middle_sub_end.y * middle_sub_end.y)
        {
            Ok(Some(Intersection::OverLap(Line2 {
                start: *start,
                end: *middle,
            })))
        } else {
            Ok(Some(Intersection::OverLap(Line2 {
                start: *middle,
                end: *end,
            })))
        }
    }

    /// returns (area of a triangle)²*4
    /// <https://en.wikipedia.org/wiki/Visvalingam–Whyatt_algorithm#Algorithm>
    pub fn triangle_area_squared_times_4(
        p1: &nalgebra::Point2<T>,
        p2: &nalgebra::Point2<T>,
        p3: &nalgebra::Point2<T>,
    ) -> T {
        let area =
            p1.x * p2.y + p2.x * p3.y + p3.x * p1.y - p1.x * p3.y - p2.x * p1.y - p3.x * p2.y;
        area * area
    }

    /// Copy this lines2 into a line3, populating the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    pub fn copy_to_3d(&self, plane: nalgebra_3d::Plane) -> nalgebra_3d::Line3<T> {
        nalgebra_3d::Line3::new(plane.point_to_3d(&self.start), plane.point_to_3d(&self.end))
    }
}

#[allow(clippy::from_over_into)]
// Todo is this a subset of "impl<T> From<[T; 4]> for Line2<T>"?
impl<T> Into<[T; 4]> for Line2<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    fn into(self) -> [T; 4] {
        [self.start.x, self.start.y, self.end.x, self.end.y]
    }
}

// [Into<Point<T>>,Into<Point<T>>] -> Line2<T>
impl<T, IT> From<[IT; 2]> for Line2<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
    IT: Copy + Into<nalgebra::Point2<T>>,
{
    fn from(coordinate: [IT; 2]) -> Line2<T> {
        Line2::<T>::new(coordinate[0].into(), coordinate[1].into())
    }
}

// [T,T,T,T] -> Line2<T>
impl<T> From<[T; 4]> for Line2<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    fn from(coordinate: [T; 4]) -> Line2<T> {
        Line2::<T>::new(
            [coordinate[0], coordinate[1]].into(),
            [coordinate[2], coordinate[3]].into(),
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
#[derive(Clone, Hash, fmt::Debug)]
pub struct VoronoiParabolicArc<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    // input geometry of voronoi graph
    pub segment: Line2<T>,
    pub cell_point: nalgebra::Point2<T>,

    // vertex points in voronoi diagram. Aka edge start and end points. (Also called circle events)
    pub start_point: nalgebra::Point2<T>,
    pub end_point: nalgebra::Point2<T>,
}

impl<T> VoronoiParabolicArc<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    pub fn new(
        segment: Line2<T>,
        cell_point: nalgebra::Point2<T>,
        start_point: nalgebra::Point2<T>,
        end_point: nalgebra::Point2<T>,
    ) -> Self {
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
    pub fn discretise_2d(&self, max_dist: T) -> LineString2<T> {
        let mut rv = LineString2::default().with_connected(false);
        rv.points.push(self.start_point);

        // Apply the linear transformation to move start point of the segment to
        // the point with coordinates (0, 0) and the direction of the segment to
        // coincide the positive direction of the x-axis.
        let segm_vec_x = self.segment.end.x - self.segment.start.x;
        let segm_vec_y = self.segment.end.y - self.segment.start.y;
        let sqr_segment_length = segm_vec_x * segm_vec_x + segm_vec_y * segm_vec_y;

        // Compute x-coordinates of the endpoints of the edge
        // in the transformed space.
        let projection_start =
            sqr_segment_length * Self::get_point_projection(&self.start_point, &self.segment);
        let projection_end =
            sqr_segment_length * Self::get_point_projection(&self.end_point, &self.segment);

        // Compute parabola parameters in the transformed space.
        // Parabola has next representation:
        // f(x) = ((x-rot_x)^2 + rot_y^2) / (2.0*rot_y).
        let point_vec_x = self.cell_point.x - self.segment.start.x;
        let point_vec_y = self.cell_point.y - self.segment.start.y;
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
                    + self.segment.start.x;
                let inter_y = (segm_vec_x * new_y + segm_vec_y * new_x) / sqr_segment_length
                    + self.segment.start.y;
                let p = nalgebra::Point2::new(inter_x, inter_y);
                rv.points.push(p);
                cur_x = new_x;
                cur_y = new_y;
            } else {
                point_stack.push(mid_x);
            }
        }

        // Update last point.
        let last_position = rv.points.len() - 1;
        rv.points[last_position] = self.end_point;
        rv
    }

    /// Convert this parable abstraction into a single straight line
    pub fn discretise_3d_straight_line(&self) -> nalgebra_3d::LineString3<T> {
        let mut rv = nalgebra_3d::LineString3::default().with_connected(false);
        let distance = -distance_to_line_squared_safe(
            &self.segment.start,
            &self.segment.end,
            &self.start_point,
        )
        .sqrt();
        rv.points
            .push([self.start_point.x, self.start_point.y, distance].into());
        let distance = -distance_to_point_squared(&self.end_point, &self.cell_point).sqrt();
        rv.points
            .push([self.end_point.x, self.end_point.y, distance].into());
        rv
    }

    /// Convert this parable abstraction into discrete line segment sample points.
    /// The Z component of the coordinates is the constant distance from the edge to point and
    /// line segment (should be the same value)
    ///
    /// All of this code is ported from C++ boost 1.75.0
    /// <https://www.boost.org/doc/libs/1_75_0/libs/polygon/doc/voronoi_main.htm>
    pub fn discretise_3d(&self, max_dist: T) -> nalgebra_3d::LineString3<T> {
        let mut rv = nalgebra_3d::LineString3::default().with_connected(false);
        let z_comp = -distance_to_point_squared(&self.start_point, &self.cell_point).sqrt();
        rv.points
            .push([self.start_point.x, self.start_point.y, z_comp].into());

        let z_comp = -distance_to_point_squared(&self.end_point, &self.cell_point).sqrt();
        // todo, don't insert end_point and then pop it again a few lines later..
        rv.points
            .push([self.end_point.x, self.end_point.y, z_comp].into());

        // Apply the linear transformation to move start point of the segment to
        // the point with coordinates (0, 0) and the direction of the segment to
        // coincide the positive direction of the x-axis.
        let segm_vec_x = self.segment.end.x - self.segment.start.x;
        let segm_vec_y = self.segment.end.y - self.segment.start.y;
        let sqr_segment_length = segm_vec_x * segm_vec_x + segm_vec_y * segm_vec_y;

        // Compute x-coordinates of the endpoints of the edge
        // in the transformed space.
        let projection_start =
            sqr_segment_length * Self::get_point_projection_3d(&rv.points[0], &self.segment);
        let projection_end =
            sqr_segment_length * Self::get_point_projection_3d(&rv.points[1], &self.segment);

        // Compute parabola parameters in the transformed space.
        // Parabola has next representation:
        // f(x) = ((x-rot_x)^2 + rot_y^2) / (2.0*rot_y).
        let point_vec_x = self.cell_point.x - self.segment.start.x;
        let point_vec_y = self.cell_point.y - self.segment.start.y;
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
                    + self.segment.start.x;
                let inter_y = (segm_vec_x * new_y + segm_vec_y * new_x) / sqr_segment_length
                    + self.segment.start.y;
                let z_comp = -distance_to_point_squared(
                    &nalgebra::Point2::new(inter_x, inter_y),
                    &self.cell_point,
                )
                .sqrt();
                rv.points
                    .push(nalgebra::Point3::new(inter_x, inter_y, z_comp));
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
    fn parabola_y(x: T, a: T, b: T) -> T {
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
    fn get_point_projection(point: &nalgebra::Point2<T>, segment: &Line2<T>) -> T {
        let segment_vec_x = segment.end.x - segment.start.x;
        let segment_vec_y = segment.end.y - segment.start.y;
        let point_vec_x = point.x - segment.start.x;
        let point_vec_y = point.y - segment.start.y;
        let sqr_segment_length = segment_vec_x * segment_vec_x + segment_vec_y * segment_vec_y;
        let vec_dot = segment_vec_x * point_vec_x + segment_vec_y * point_vec_y;
        vec_dot / sqr_segment_length
    }

    // exactly the same as get_point_projection but with a Point3 (Z component will be ignored)
    fn get_point_projection_3d(point: &nalgebra::Point3<T>, segment: &Line2<T>) -> T {
        let segment_vec_x = segment.end.x - segment.start.x;
        let segment_vec_y = segment.end.y - segment.start.y;
        let point_vec_x = point.x - segment.start.x;
        let point_vec_y = point.y - segment.start.y;
        let sqr_segment_length = segment_vec_x * segment_vec_x + segment_vec_y * segment_vec_y;
        let vec_dot = segment_vec_x * point_vec_x + segment_vec_y * point_vec_y;
        vec_dot / sqr_segment_length
    }
}

/// A 2d line string, aka polyline.
/// If the 'connected' field is set the 'as_lines()' method will connect start point with the
/// end-point.
/// Todo: The builder structure of this struct needs to be revisited
#[derive(PartialEq, Eq, Clone, Hash, fmt::Debug)]
pub struct LineString2<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    pub(crate) points: Vec<nalgebra::Point2<T>>,

    /// if connected is set the as_lines() method will add an extra line connecting
    /// the first and last point
    pub connected: bool,
}

impl<T> Default for LineString2<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    #[inline]
    fn default() -> Self {
        Self {
            points: Vec::<nalgebra::Point2<T>>::new(),
            connected: false,
        }
    }
}

struct PriorityDistance<T> {
    key: T,
    index: usize,
}

impl<T> PartialOrd for PriorityDistance<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl<T> Ord for PriorityDistance<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    #[inline(always)]
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.key.partial_cmp(&other.key).unwrap().reverse()
    }
}

impl<T> PartialEq for PriorityDistance<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        ulps_eq!(&self.key, &other.key)
    }
}
impl<T> Eq for PriorityDistance<T> where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast
{
}

impl<T> LineString2<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::<nalgebra::Point2<T>>::with_capacity(capacity),
            connected: false,
        }
    }

    pub fn with_points(mut self, points: Vec<nalgebra::Point2<T>>) -> Self {
        self.points = points;
        self
    }

    pub fn with_connected(mut self, connected: bool) -> Self {
        self.connected = connected;
        self
    }

    /// Copies the points of the iterator into the LineString2
    /// from_iter is already claimed for into() objects.
    pub fn with_iter<'a, I>(iter: I) -> Self
    where
        T: 'a,
        I: Iterator<Item = &'a nalgebra::Point2<T>>,
    {
        Self {
            points: iter.into_iter().copied().collect(),
            connected: false,
        }
    }

    /// Moves all the elements of `other` into `Self`, leaving `other` empty.
    /// TODO: currently ignores if `other` is connected or not.
    /// # Panics
    /// Panics if the number of elements in the points vector overflows a `usize`.
    pub fn append(&mut self, mut other: Self) {
        self.points.append(&mut other.points);
    }

    /// Returns true if the lines are self intersecting
    /// If number of points < 10 then the intersections are tested using brute force O(n²)
    /// If more than that a sweep-line algorithm is used O(n*log(n)+i*log(n))
    pub fn is_self_intersecting(&self) -> Result<bool, LinestringError> {
        if self.points.len() <= 2 {
            Ok(false)
        } else if self.points.len() < 10 {
            //let lines = self.as_lines_iter();
            for l0 in self
                .as_lines_iter()
                .enumerate()
                .take(self.as_lines_iter_len() - 1)
            {
                for l1 in self.as_lines_iter().enumerate().skip(l0.0 + 1) {
                    if l0.0 == l1.0 {
                        continue;
                    } else if l0.0 + 1 == l1.0 {
                        // consecutive line: l0.0.start <-> l0.0.end_point <-> l2.0.end_point
                        if Line2::intersection_point3(&l0.1.start, &l0.1.end, &l1.1.end)?.is_some()
                        {
                            return Ok(true);
                        }
                    } else if let Some(point) = l0.1.intersection_point(&l1.1) {
                        let point = point.single();
                        if (point_ulps_eq(&point, &l0.1.start) || point_ulps_eq(&point, &l0.1.end))
                            && (point_ulps_eq(&point, &l1.1.start)
                                || point_ulps_eq(&point, &l1.1.end))
                        {
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
            let result = intersection::IntersectionData::<T>::default()
                .with_ignore_end_point_intersections(true)?
                .with_stop_at_first_intersection(true)?
                .with_lines(self.as_lines_iter())?
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

    pub fn points(&self) -> &Vec<nalgebra::Point2<T>> {
        &self.points
    }

    /// returns the number of points in the point list
    /// not the number of segments (add one if connected)
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// returns true if the point list is empty
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Returns the line string as a Vec of lines
    /// Will be deprecated at some time, use self.as_lines_iter().collect() instead
    #[inline(always)]
    pub fn as_lines(&self) -> Vec<Line2<T>> {
        self.as_lines_iter().collect()
    }

    /// Returns the line string as a iterator of lines
    #[must_use = "iterator adaptors are lazy and do nothing unless consumed"]
    pub fn as_lines_iter<'a>(&'a self) -> Box<dyn Iterator<Item = Line2<T>> + 'a> {
        if self.connected {
            Box::new(
                self.points
                    .iter()
                    .chain(self.points.first())
                    .tuple_windows::<(_, _)>()
                    .map(|(a, b)| Line2 { start: *a, end: *b }),
            )
        } else {
            Box::new(
                self.points
                    .iter()
                    .tuple_windows::<(_, _)>()
                    .map(|(a, b)| Line2 { start: *a, end: *b }),
            )
        }
    }

    /// The iterator of as_lines_iter() does not implement ExactSizeIterator.
    /// This can be used as a work around for that problem
    pub fn as_lines_iter_len(&self) -> usize {
        if self.points.len() < 2 {
            return 0;
        }
        if self.connected {
            self.points.len()
        } else {
            self.points.len() - 1
        }
    }

    /// Copy this linestring2 into a linestring3, populating the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    pub fn copy_to_3d(&self, plane: nalgebra_3d::Plane) -> nalgebra_3d::LineString3<T> {
        let mut rv: nalgebra_3d::LineString3<T> = match plane {
            nalgebra_3d::Plane::XY => self
                .points
                .iter()
                .map(|p2d| nalgebra::Point3::new(p2d.x, p2d.y, T::zero()))
                .collect(),
            nalgebra_3d::Plane::XZ => self
                .points
                .iter()
                .map(|p2d| nalgebra::Point3::new(p2d.x, T::zero(), p2d.y))
                .collect(),
            nalgebra_3d::Plane::ZY => self
                .points
                .iter()
                .map(|p2d| nalgebra::Point3::new(T::zero(), p2d.y, p2d.x))
                .collect(),
        };
        rv.connected = self.connected;
        rv
    }

    pub fn push(&mut self, point: nalgebra::Point2<T>) {
        self.points.push(point);
    }

    pub fn transform(&self, matrix3x3: &nalgebra::Matrix3<T>) -> Self {
        Self {
            points: self
                .points
                .iter()
                .map(|x| matrix3x3.transform_point(x))
                .collect(),
            connected: self.connected,
        }
    }

    /// Simplify using Ramer–Douglas–Peucker algorithm
    pub fn simplify(&self, distance_predicate: T) -> Self {
        //println!("input dist:{:?} slice{:?}", distance_predicate, self.points);

        if self.points.len() <= 2 {
            return self.clone();
        }
        if self.connected {
            let mut points = self.points.clone();
            // add the start-point to the end
            points.push(*points.first().unwrap());

            let mut rv: Vec<nalgebra::Point2<T>> = Vec::with_capacity(points.len());
            // _simplify() always omits the the first point of the result, so we have to add that
            rv.push(*points.first().unwrap());
            rv.append(&mut Self::_simplify(
                distance_predicate * distance_predicate,
                points.as_slice(),
            ));
            // remove the start-point from the the end
            let _ = rv.remove(rv.len() - 1);
            Self {
                points: rv,
                connected: true,
            }
        } else {
            let mut rv: Vec<nalgebra::Point2<T>> = Vec::with_capacity(self.points.len());
            // _simplify() always omits the the first point of the result, so we have to add that
            rv.push(*self.points.first().unwrap());
            rv.append(&mut Self::_simplify(
                distance_predicate * distance_predicate,
                self.points.as_slice(),
            ));
            Self {
                points: rv,
                connected: false,
            }
        }
    }

    /// A naïve recursive implementation of Ramer–Douglas–Peucker algorithm
    /// It spawns a lot of Vec, but it seems to work
    fn _simplify(
        distance_predicate_sq: T,
        slice: &[nalgebra::Point2<T>],
    ) -> Vec<nalgebra::Point2<T>> {
        //println!("input dist:{:?} slice{:?}", distance_predicate_sq, slice);
        if slice.len() <= 2 {
            return slice[1..].to_vec();
        }
        // unwrap is safe since we tested len()>2
        let start_point = slice.first().unwrap();
        let end_point = slice.last().unwrap();

        let mut max_dist_sq = (-T::one(), 0_usize);
        let mut found_something = false;
        // find the point with largest distance to start_point<->endpoint line
        for (i, point) in slice.iter().enumerate().take(slice.len() - 1).skip(1) {
            let sq_d = distance_to_line_squared_safe(start_point, end_point, point);

            //println!("sq_d:{:?}", sq_d);
            if sq_d > max_dist_sq.0 && sq_d > distance_predicate_sq {
                max_dist_sq = (sq_d, i);
                found_something = true;
            }
        }

        //println!("max_dist_sq: {:?}", max_dist_sq);
        if !found_something {
            // no point was outside the distance limit, return a new list only containing the
            // end point (start point is implicit)
            //println!("return start-end");
            return vec![*end_point];
        }

        let mut rv = Self::_simplify(distance_predicate_sq, &slice[..max_dist_sq.1 + 1]);
        rv.append(&mut Self::_simplify(
            distance_predicate_sq,
            &slice[max_dist_sq.1..],
        ));
        rv
    }

    /// Simplify using Visvalingam–Whyatt algorithm. This algorithm will delete 'points_to_delete'
    /// of points from the polyline with the smallest area defined by one point and it's neighbours.
    pub fn simplify_vw(&self, points_to_delete: usize) -> Self {
        if (self.connected && self.points.len() <= 1) || (!self.connected && self.points.len() <= 2)
        {
            // Nothing to do here, we can't delete endpoints if not connected,
            // and we must leave at least one point if connected.
            return self.clone();
        }
        // priority queue key: area, value: indices of self.points + a copy of area.
        // When a point is removed it's previously calculated area-to-neighbour value will not be
        // removed. Instead new areas will simply be added to the priority queue.
        // If a removed node is pop():ed it will be checked against the link_tree hash map.
        // If it is not in there or if the area doesn't match it will simply be ignored and a
        // new value pop():ed.

        let mut search_tree = collections::binary_heap::BinaryHeap::<PriorityDistance<T>>::new();
        // map from node number to remaining neighbours of that node. All indices of self.points
        // AHashMap::<node_id:usize, (prev_node_id:usize, next_node_id:usize, area:T)>
        let mut link_tree = ahash::AHashMap::<usize, (usize, usize, T)>::default();
        {
            let mut iter_i = self.points.iter().enumerate();
            let mut iter_j = self.points.iter().enumerate().skip(1);
            // the k iterator will terminate before i & j, so the iter_i & iter_j unwrap()s are safe
            for k in self.points.iter().enumerate().skip(2) {
                let i = iter_i.next().unwrap();
                let j = iter_j.next().unwrap();
                // define the area between point i, j & k as search criteria
                let area = Line2::triangle_area_squared_times_4(i.1, j.1, k.1);
                let _ = search_tree.push(PriorityDistance {
                    key: area,
                    index: j.0,
                });
                // point j is connected to point i and k
                let _ = link_tree.insert(j.0, (i.0, k.0, area));
            }
        }
        if self.connected {
            // add an extra point at the end, faking the loop
            let i = self.points.len() - 2;
            let j = self.points.len() - 1;
            let k = self.points.len();
            let area = Line2::triangle_area_squared_times_4(
                &self.points[i],
                &self.points[j],
                &self.points[0],
            );
            let _ = search_tree.push(PriorityDistance {
                key: area,
                index: j,
            });
            let _ = link_tree.insert(j, (i, k, area));
        }

        let self_points_len = self.points.len();

        let mut deleted_nodes: usize = 0;
        loop {
            if search_tree.is_empty() || deleted_nodes >= points_to_delete {
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
                                &self.points[prev],
                                &self.points[next % self_points_len],
                                &self.points[next_next % self_points_len],
                            );
                            let _ = search_tree.push(PriorityDistance {
                                key: area,
                                index: next,
                            });
                            let _ = link_tree.insert(next, (prev, next_next, area));

                            let area = Line2::triangle_area_squared_times_4(
                                &self.points[prev_prev],
                                &self.points[prev],
                                &self.points[next % self_points_len],
                            );
                            let _ = search_tree.push(PriorityDistance {
                                key: area,
                                index: prev,
                            });
                            let _ = link_tree.insert(prev, (prev_prev, next, area));
                            continue;
                        }
                    }

                    if let Some(prev_prev) = prev_prev {
                        let area = Line2::triangle_area_squared_times_4(
                            &self.points[prev_prev],
                            &self.points[prev],
                            &self.points[next % self_points_len],
                        );
                        let _ = search_tree.push(PriorityDistance {
                            key: area,
                            index: prev,
                        });
                        let _ = link_tree.insert(prev, (prev_prev, next, area));
                        continue;
                    };

                    if let Some(next_next) = next_next {
                        let area = Line2::triangle_area_squared_times_4(
                            &self.points[prev],
                            &self.points[next % self_points_len],
                            &self.points[next_next % self_points_len],
                        );
                        let _ = search_tree.push(PriorityDistance {
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
        if !self.connected {
            [0_usize]
                .iter()
                .copied()
                .chain(
                    link_tree
                        .keys()
                        .sorted_unstable()
                        .copied()
                        .chain([self.points.len() - 1].iter().copied()),
                )
                .map(|x| self.points[x])
                .collect::<Self>()
                .with_connected(false)
        } else {
            [0_usize]
                .iter()
                .copied()
                .chain(link_tree.keys().sorted_unstable().copied())
                .map(|x| self.points[x])
                .collect::<Self>()
                .with_connected(true)
        }
    }

    pub fn operation<F>(&mut self, f: F)
    where
        F: Fn(T) -> T,
    {
        for v in self.points.iter_mut() {
            v.x = f(v.x);
            v.y = f(v.y);
        }
    }
}

impl<T> From<Aabb2<T>> for LineString2<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    /// creates a connected LineString2 from the outlines of the Aabb2
    fn from(other: Aabb2<T>) -> Self {
        if let Some(min_max) = other.min_max {
            let points = vec![
                min_max.0,
                nalgebra::Point2::new(min_max.1.x, min_max.0.y),
                min_max.1,
                nalgebra::Point2::new(min_max.0.x, min_max.1.y),
            ];
            Self {
                points,
                connected: true,
            }
        } else {
            Self {
                points: Vec::<nalgebra::Point2<T>>::new(),
                connected: false,
            }
        }
    }
}

impl<T, IC> std::iter::FromIterator<IC> for LineString2<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
    IC: Into<nalgebra::Point2<T>>,
{
    fn from_iter<I: IntoIterator<Item = IC>>(iter: I) -> Self {
        LineString2 {
            points: iter.into_iter().map(|c| c.into()).collect(),
            connected: false,
        }
    }
}

/// A set of 2d LineString, an aabb + convex_hull.
/// It also contains a list of aabb & convex_hulls of shapes this set has gobbled up.
/// This can be useful for separating out inner regions of the shape.
///
/// This struct is intended to contain related shapes. E.g. outlines of letters with holes
#[derive(PartialEq, Eq, Clone, Hash)]
pub struct LineStringSet2<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    set: Vec<LineString2<T>>,
    aabb: Aabb2<T>,
    convex_hull: Option<LineString2<T>>,
    pub internals: Option<Vec<(Aabb2<T>, LineString2<T>)>>,
}

impl<T> Default for LineStringSet2<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    #[inline]
    fn default() -> Self {
        Self {
            set: Vec::<LineString2<T>>::new(),
            aabb: Aabb2::default(),
            convex_hull: None,
            internals: None,
        }
    }
}

impl<T> LineStringSet2<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    /// steal the content of 'other' leaving it empty
    pub fn steal_from(other: &mut LineStringSet2<T>) -> Self {
        //println!("stealing from other.aabb:{:?}", other.aabb);
        let mut set = Vec::<LineString2<T>>::new();
        set.append(&mut other.set);
        Self {
            set,
            aabb: other.aabb.clone(),
            convex_hull: other.convex_hull.take(),
            internals: other.internals.take(),
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

            for ls in self.set.last().unwrap().points.iter() {
                self.aabb.update_point(ls);
            }
        }
    }

    /// returns the combined convex hull of all the shapes in self.set
    pub fn get_convex_hull(&self) -> &Option<LineString2<T>> {
        &self.convex_hull
    }

    /// calculates the combined convex hull of all the shapes in self.set
    pub fn calculate_convex_hull(&mut self) -> &LineString2<T> {
        self.convex_hull = Some(convex_hull::ConvexHull::graham_scan(
            self.set.iter().map(|x| x.points()).flatten(),
        ));
        self.convex_hull.as_ref().unwrap()
    }

    /// Returns the axis aligned bounding box of this set.
    pub fn get_aabb(&self) -> &Aabb2<T> {
        &self.aabb
    }

    /// Transform each individual component of this set using the transform matrix.
    /// Return the result in a new object.
    pub fn transform(&self, matrix3x3: &nalgebra::Matrix3<T>) -> Self {
        let internals = self.internals.as_ref().map(|internals| {
            internals
                .iter()
                .map(|x| (x.0.transform(matrix3x3), x.1.transform(matrix3x3)))
                .collect()
        });

        let convex_hull = self
            .convex_hull
            .as_ref()
            .map(|convex_hull| convex_hull.transform(matrix3x3));

        Self {
            aabb: self.aabb.transform(matrix3x3),
            set: self.set.iter().map(|x| x.transform(matrix3x3)).collect(),
            convex_hull,
            internals,
        }
    }

    /// Copy this linestringset2 into a linestringset3, populating the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    /// The empty axis will be set to zero.
    pub fn copy_to_3d(&self, plane: nalgebra_3d::Plane) -> nalgebra_3d::LineStringSet3<T> {
        let mut rv = nalgebra_3d::LineStringSet3::with_capacity(self.set.len());
        for ls in self.set.iter() {
            rv.push(ls.copy_to_3d(plane));
        }
        rv
    }

    /// drains the 'other' container of all shapes and put them into 'self'
    pub fn take_from(&mut self, mut other: Self) {
        self.aabb.update_aabb(&other.aabb);
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
        if self.aabb.get_low().is_none() {
            //println!("self.aabb {:?}", self.aabb);
            //println!("other.aabb {:?}", other.aabb);
            return Err(LinestringError::InvalidData(
                "'self' did not contain a valid 'aabb' field".to_string(),
            ));
        }
        if other.aabb.get_low().is_none() {
            return Err(LinestringError::InvalidData(
                "'other' did not contain a valid 'aabb' field".to_string(),
            ));
        }
        if !self.aabb.contains_aabb(&other.aabb) {
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
            .push((other.aabb.clone(), other.convex_hull.take().unwrap()));
        Ok(())
    }

    /// Iterate an operation over each coordinate in the contained objects.
    /// Useful when you want to round the value of each contained coordinate.
    pub fn operation<F>(&mut self, f: &F)
    where
        F: Fn(T) -> T,
    {
        for s in self.set.iter_mut() {
            s.operation(f);
        }
        self.aabb.operation(f);
        if let Some(ref mut convex_hull) = self.convex_hull {
            convex_hull.operation(f);
        }
        if let Some(ref mut internals) = self.internals {
            for i in internals.iter_mut() {
                i.0.operation(f);
                i.1.operation(f);
            }
        }
    }
}

/// A simple 2d AABB
/// If min_max is none the data has not been assigned yet.
#[derive(PartialEq, Eq, Clone, Hash, fmt::Debug)]
pub struct Aabb2<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    min_max: Option<(nalgebra::Point2<T>, nalgebra::Point2<T>)>,
}

impl<T, IT> From<[IT; 2]> for Aabb2<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
    IT: Copy + Into<nalgebra::Point2<T>>,
{
    fn from(coordinate: [IT; 2]) -> Aabb2<T> {
        let mut rv = Aabb2::<T>::default();
        rv.update_point(&coordinate[0].into());
        rv.update_point(&coordinate[1].into());
        rv
    }
}

impl<T> From<[T; 4]> for Aabb2<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    fn from(coordinate: [T; 4]) -> Aabb2<T> {
        let mut rv = Aabb2::default();
        rv.update_point(&nalgebra::Point2::new(coordinate[0], coordinate[1]));
        rv.update_point(&nalgebra::Point2::new(coordinate[2], coordinate[3]));
        rv
    }
}

impl<T> Default for Aabb2<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    #[inline]
    fn default() -> Self {
        Self { min_max: None }
    }
}

impl<T> Aabb2<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    pub fn new(point: &nalgebra::Point2<T>) -> Self {
        Self {
            min_max: Some((*point, *point)),
        }
    }

    pub fn update_aabb(&mut self, aabb: &Aabb2<T>) {
        if let Some((min, max)) = &aabb.min_max {
            self.update_point(min);
            self.update_point(max);
        }
    }

    pub fn update_point(&mut self, point: &nalgebra::Point2<T>) {
        if self.min_max.is_none() {
            self.min_max = Some((*point, *point));
            return;
        }
        let (mut aabb_min, mut aabb_max) = self.min_max.take().unwrap();

        if point.x < aabb_min.x {
            aabb_min.x = point.x;
        }
        if point.y < aabb_min.y {
            aabb_min.y = point.y;
        }
        if point.x > aabb_max.x {
            aabb_max.x = point.x;
        }
        if point.y > aabb_max.y {
            aabb_max.y = point.y;
        }
        self.min_max = Some((aabb_min, aabb_max));
    }

    pub fn get_high(&self) -> Option<nalgebra::Point2<T>> {
        if let Some((_, _high)) = self.min_max {
            return Some(_high);
        }
        None
    }

    pub fn get_low(&self) -> Option<nalgebra::Point2<T>> {
        if let Some((_low, _)) = self.min_max {
            return Some(_low);
        }
        None
    }

    pub fn transform(&self, matrix3x3: &nalgebra::Matrix3<T>) -> Self {
        if let Some(min_max) = self.min_max {
            Self {
                min_max: Some((
                    matrix3x3.transform_point(&min_max.0),
                    matrix3x3.transform_point(&min_max.1),
                )),
            }
        } else {
            Self { min_max: None }
        }
    }

    /// returns true if this aabb entirely contains/engulfs 'other' (inclusive)
    #[inline(always)]
    pub fn contains_aabb(&self, other: &Aabb2<T>) -> bool {
        if let Some(self_aabb) = self.min_max {
            if let Some(other_aabb) = other.min_max {
                return Self::contains_point_inclusive_(&self_aabb, &other_aabb.0)
                    && Self::contains_point_inclusive_(&self_aabb, &other_aabb.1);
            }
        }
        false
    }

    /// returns true if this aabb entirely contains/engulfs a line (inclusive)
    #[inline(always)]
    pub fn contains_line_inclusive(&self, line: &Line2<T>) -> bool {
        if let Some(self_aabb) = self.min_max {
            return Self::contains_point_inclusive_(&self_aabb, &line.start)
                && Self::contains_point_inclusive_(&self_aabb, &line.end);
        }
        false
    }

    /// returns true if this aabb contains a point (inclusive)
    #[inline(always)]
    pub fn contains_point_inclusive(&self, point: &nalgebra::Point2<T>) -> bool {
        if let Some(self_aabb) = self.min_max {
            return Self::contains_point_inclusive_(&self_aabb, point);
        }
        false
    }

    /// returns true if aabb contains a point (inclusive)
    #[inline(always)]
    fn contains_point_inclusive_(
        aabb: &(nalgebra::Point2<T>, nalgebra::Point2<T>),
        point: &nalgebra::Point2<T>,
    ) -> bool {
        (aabb.0.x <= point.x || ulps_eq!(&aabb.0.x, &point.x))
            && (aabb.0.y <= point.y || ulps_eq!(&aabb.0.y, &point.y))
            && (aabb.1.x >= point.x || ulps_eq!(&aabb.1.x, &point.x))
            && (aabb.1.y >= point.y || ulps_eq!(&aabb.1.y, &point.y))
    }

    pub fn operation<F>(&mut self, f: F)
    where
        F: Fn(T) -> T,
    {
        if let Some(ref mut min_max) = self.min_max {
            min_max.0.x = f(min_max.0.x);
            min_max.0.y = f(min_max.0.y);
            min_max.1.x = f(min_max.1.x);
            min_max.1.y = f(min_max.1.y);
        }
    }
}

/// Get any intersection point between line segment and point.
/// Inspired by <https://stackoverflow.com/a/17590923>
pub fn intersect_line_point<T>(
    line: &Line2<T>,
    point: &nalgebra::Point2<T>,
) -> Option<Intersection<T>>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    // take care of end point equality
    if ulps_eq!(&line.start.x, &point.x) && ulps_eq!(&line.start.y, &point.y) {
        return Some(Intersection::Intersection(*point));
    }
    if ulps_eq!(&line.end.x, &point.x) && ulps_eq!(&line.end.y, &point.y) {
        return Some(Intersection::Intersection(*point));
    }

    let x1 = line.start.x;
    let x2 = line.end.x;
    let y1 = line.start.y;
    let y2 = line.end.y;
    let x = point.x;
    let y = point.y;

    let ab = ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)).sqrt();
    let ap = ((x - x1) * (x - x1) + (y - y1) * (y - y1)).sqrt();
    let pb = ((x2 - x) * (x2 - x) + (y2 - y) * (y2 - y)).sqrt();

    #[cfg(feature = "console_trace")]
    println!("ab={:?}, ap={:?}, pb={:?}, ap+pb={:?}", ab, ap, pb, ap + pb);
    if ulps_eq!(&ab, &(ap + pb)) {
        return Some(Intersection::Intersection(*point));
    }
    None
}

#[allow(dead_code)]
pub enum Intersection<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    // Normal one point intersection
    Intersection(nalgebra::Point2<T>),
    // Collinear overlapping
    OverLap(Line2<T>),
}

impl<T> Intersection<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    /// return a single, simple intersection point
    pub fn single(&self) -> nalgebra::Point2<T> {
        match self {
            Self::OverLap(a) => a.start,
            Self::Intersection(a) => *a,
        }
    }
}

impl<T> fmt::Debug for Intersection<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::OverLap(a) => a.fmt(f),
            Self::Intersection(a) => a.fmt(f),
        }
    }
}

#[inline(always)]
pub fn scale_to_coordinate<T>(
    point: &nalgebra::Point2<T>,
    vector: &nalgebra::Vector2<T>,
    scale: T,
) -> nalgebra::Point2<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    nalgebra::Point2::new(point.x + scale * vector.x, point.y + scale * vector.y)
}

#[inline(always)]
/// Divides a 'vector' by 'b'. Obviously, don't feed this with 'b' == 0
fn div<T>(a: &nalgebra::Vector2<T>, b: T) -> nalgebra::Vector2<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    nalgebra::Vector2::<T>::new(a.x / b, a.y / b)
}

#[inline(always)]
/// subtracts point b from point a resulting in a vector
/// This is a compatibility workaround for 2d packages without any math
fn sub<T>(a: &nalgebra::Point2<T>, b: &nalgebra::Point2<T>) -> nalgebra::Vector2<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    a - b
}

#[inline(always)]
/// from <https://stackoverflow.com/a/565282> :
///  "Define the 2-dimensional vector cross product v × w to be vx wy − vy wx."
/// This function returns the z component of v × w (if we pretend v and w are two dimensional)
fn cross_z<T>(v: &nalgebra::Vector2<T>, w: &nalgebra::Vector2<T>) -> T
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    v.x * w.y - v.y * w.x
}

#[inline(always)]
/// The distance between the line a->b to the point p is the same as
/// distance = |(a-p)×(a-b)|/|a-b|
/// <https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Another_vector_formulation>
/// Make sure to *not* call this function with a-b==0
/// This function returns the distance²
pub fn distance_to_line_squared<T>(
    a: &nalgebra::Point2<T>,
    b: &nalgebra::Point2<T>,
    p: &nalgebra::Point2<T>,
) -> T
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    let a_sub_b = sub(a, b);
    let a_sub_p = sub(a, p);
    let a_sub_p_cross_a_sub_b = cross_z(&a_sub_p, &a_sub_b);
    (a_sub_p_cross_a_sub_b * a_sub_p_cross_a_sub_b)
        / (a_sub_b.x * a_sub_b.x + a_sub_b.y * a_sub_b.y)
}

/// Same as distance_to_line_squared<T> but it can be called when a-b might be 0.
/// It's a little slower because it does the a==b test
#[inline(always)]
pub fn distance_to_line_squared_safe<T>(
    a: &nalgebra::Point2<T>,
    b: &nalgebra::Point2<T>,
    p: &nalgebra::Point2<T>,
) -> T
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    if point_ulps_eq(a, b) {
        // give the point-to-point answer if the segment is a point
        distance_to_point_squared(a, p)
    } else {
        let a_sub_b = sub(a, b);
        let a_sub_p = sub(a, p);
        let a_sub_p_cross_a_sub_b = cross_z(&a_sub_p, &a_sub_b);
        (a_sub_p_cross_a_sub_b * a_sub_p_cross_a_sub_b)
            / (a_sub_b.x * a_sub_b.x + a_sub_b.y * a_sub_b.y)
    }
}

#[inline(always)]
/// The distance² between the two points
pub fn distance_to_point_squared<T>(a: &nalgebra::Point2<T>, b: &nalgebra::Point2<T>) -> T
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    let v = sub(a, b);
    v.x * v.x + v.y * v.y
}

#[inline(always)]
/// calculate the dot product of two vectors
fn dot<T>(a: &nalgebra::Vector2<T>, b: &nalgebra::Vector2<T>) -> T
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    a.x * b.x + a.y * b.y
}

#[inline(always)]
pub fn point_ulps_eq<T>(a: &nalgebra::Point2<T>, b: &nalgebra::Point2<T>) -> bool
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    ulps_eq!(&a.x, &b.x) && ulps_eq!(&a.y, &b.y)
}

/// This is a simple but efficient affine transformation object.
/// It can pan, zoom and flip points around center axis but not rotate.
/// It does not handle vector transformation, only points.
#[derive(PartialEq, Clone, fmt::Debug)]
pub struct SimpleAffine<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    /// The offsets used to center the 'source' coordinate system. Typically the input geometry
    /// in this case.
    pub a_offset: [T; 2],
    /// A zoom scale
    pub scale: [T; 2],
    /// The offsets needed to center coordinates of interest on the 'dest' coordinate system.
    /// i.e. the screen coordinate system.
    pub b_offset: [T; 2],
}

impl<T> Default for SimpleAffine<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    #[inline]
    fn default() -> Self {
        Self {
            a_offset: [T::zero(), T::zero()],
            scale: [T::one(), T::one()],
            b_offset: [T::zero(), T::zero()],
        }
    }
}

impl<T> SimpleAffine<T>
where
    T: nalgebra::RealField
        + Sync
        + AbsDiffEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + RelativeEq<Epsilon = T>
        + UlpsEq<Epsilon = T>
        + num_traits::cast::NumCast,
{
    pub fn new(a_aabb: &Aabb2<T>, b_aabb: &Aabb2<T>) -> Result<Self, LinestringError> {
        let min_dim = T::from(1.0).unwrap();
        let two = T::from(2.0).unwrap();

        if let Some(source_low) = a_aabb.get_low() {
            if let Some(source_high) = a_aabb.get_high() {
                if let Some(destination_low) = b_aabb.get_low() {
                    if let Some(destination_high) = b_aabb.get_high() {
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
    ///```
    /// # use linestring::nalgebra_2d;
    /// type T = f32;
    ///
    /// // source is (-100,-100)-(100,100)
    /// let mut aabb_source = nalgebra_2d::Aabb2::<T>::from([-100.,-100.,100.,100.]);
    /// // dest is (0,0)-(800,800.)
    /// let mut aabb_dest = nalgebra_2d::Aabb2::<T>::from([0.,0.,800.,800.]);
    /// let transform = nalgebra_2d::SimpleAffine::new(&aabb_source, &aabb_dest).unwrap();
    ///
    /// assert_eq!(
    ///   transform.transform_ab(&nalgebra::Point2::new(-100.,-100.)).unwrap(),
    ///   nalgebra::Point2::new(0.,0.)
    ///  );
    ///  assert_eq!(
    ///  transform.transform_ba(&nalgebra::Point2::new(0.,0.)).unwrap(),
    ///    nalgebra::Point2::new(-100.,-100.)
    ///  );
    ///  assert_eq!(
    ///    transform.transform_ab(&nalgebra::Point2::new(100.,100.)).unwrap(),
    ///    nalgebra::Point2::new(800.,800.)
    ///  );
    ///  assert_eq!(
    ///    transform.transform_ba(&nalgebra::Point2::new(800.,800.)).unwrap(),
    ///    nalgebra::Point2::new(100.,100.)
    ///  );
    ///```
    #[inline(always)]
    pub fn transform_ba(
        &self,
        point: &nalgebra::Point2<T>,
    ) -> Result<nalgebra::Point2<T>, LinestringError> {
        let x = (point.x - self.b_offset[0]) / self.scale[0] - self.a_offset[0];
        let y = (point.y - self.b_offset[1]) / self.scale[1] - self.a_offset[1];
        if x.is_finite() && y.is_finite() {
            Ok(nalgebra::Point2::new(x, y))
        } else {
            Err(LinestringError::TransformError(
                "Transformation out of bounds".to_string(),
            ))
        }
    }

    /// Transform from source (a) coordinate system to dest (b) coordinate system
    ///```
    /// # use linestring::nalgebra_2d;
    /// type T = f32;
    /// // source is (0,0)-(1,1)
    /// let mut aabb_source = nalgebra_2d::Aabb2::<T>::from([0.,0.,1.,1.]);
    /// // dest is (1,1)-(2,2)
    /// let mut aabb_dest = nalgebra_2d::Aabb2::<T>::from([1.,1.,2.,2.]);
    ///
    /// let transform = nalgebra_2d::SimpleAffine::new(&aabb_source, &aabb_dest).unwrap();
    /// assert_eq!(
    ///   transform.transform_ab(&nalgebra::Point2::new(0.,0.)).unwrap(),
    ///    nalgebra::Point2::new(1.,1.)
    ///  );
    /// assert_eq!(
    /// transform.transform_ab(&nalgebra::Point2::new(1.,1.)).unwrap(),
    ///   nalgebra::Point2::new(2.,2.)
    /// );
    /// assert_eq!(
    ///   transform.transform_ab(&nalgebra::Point2::new(0.,1.)).unwrap(),
    ///   nalgebra::Point2::new(1.,2.)
    /// );
    /// assert_eq!(
    ///   transform.transform_ab(&nalgebra::Point2::new(1.,0.)).unwrap(),
    ///   nalgebra::Point2::new(2.,1.)
    /// );
    ///```
    #[inline(always)]
    pub fn transform_ab(
        &self,
        point: &nalgebra::Point2<T>,
    ) -> Result<nalgebra::Point2<T>, LinestringError> {
        let x = (point.x + self.a_offset[0]) * self.scale[0] + self.b_offset[0];
        let y = (point.y + self.a_offset[1]) * self.scale[1] + self.b_offset[1];
        if x.is_finite() && y.is_finite() {
            Ok(nalgebra::Point2::new(x, y))
        } else {
            Err(LinestringError::TransformError(
                "Transformation out of bounds".to_string(),
            ))
        }
    }

    /// transform an array from dest (b) coordinate system to source (a) coordinate system
    #[inline(always)]
    pub fn transform_ba_a(&self, points: [T; 4]) -> Result<[T; 4], LinestringError> {
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
    pub fn transform_ab_a(&self, points: [T; 4]) -> Result<[T; 4], LinestringError> {
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
