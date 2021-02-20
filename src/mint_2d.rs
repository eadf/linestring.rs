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

use super::mint_3d;
#[allow(unused_imports)]
use crate::LinestringError;

use std::fmt;

#[cfg(feature = "impl-mint")]
pub mod intersection;

/// A 2d line
#[derive(PartialEq, Eq, Copy, Clone, Hash, fmt::Debug)]
pub struct Line2<T>
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    pub start: mint::Point2<T>,
    pub end: mint::Point2<T>,
}

impl<T> Line2<T>
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    pub fn new(start: mint::Point2<T>, end: mint::Point2<T>) -> Self {
        Self { start, end }
    }

    /// Get any intersection point between line segments.
    /// Note that this function always detects endpoint-to-endpoint intersections.
    /// Most of this is from https://stackoverflow.com/a/565282
    #[allow(clippy::many_single_char_names)]
    pub fn intersection_point(&self, other: &Self) -> Option<Intersection<T>>
    where
        T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
    {
        let p = self.start;
        let q = other.start;
        let r = sub(&self.end, &p);
        let s = sub(&other.end, &q);

        let r_cross_s = cross_z(&r, &s);
        let q_minus_p = sub(&q, &p);
        let q_minus_p_cross_r = cross_z(&q_minus_p, &r);

        // If r × s = 0 then the two lines are parallel
        if ulps_eq(&r_cross_s, &T::zero()) {
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
            if ulps_eq(&q_minus_p_cross_r, &T::zero()) {
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
    pub fn intersection_point3(
        start: &mint::Point2<T>,
        middle: &mint::Point2<T>,
        end: &mint::Point2<T>,
    ) -> Result<Option<Intersection<T>>, LinestringError> {
        let middle_sub_start = mint::Vector2 {
            x: middle.x - start.x,
            y: middle.y - start.x,
        };
        let middle_sub_end = mint::Vector2 {
            x: middle.x - end.x,
            y: middle.y - end.y,
        };
        let start_is_vertical = ulps_eq(&middle_sub_start.x, &T::zero());
        let end_is_vertical = ulps_eq(&middle_sub_end.x, &T::zero());

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

        if !ulps_eq(&start_slope, &end_slope) {
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
}

#[allow(clippy::from_over_into)]
// Todo is this a subset of "impl<T> From<[T; 4]> for Line2<T>"?
impl<T> Into<[T; 4]> for Line2<T>
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    fn into(self) -> [T; 4] {
        [self.start.x, self.start.y, self.end.x, self.end.y]
    }
}

// [Into<Point<T>>,Into<Point<T>>] -> Line2<T>
impl<T, IT> From<[IT; 2]> for Line2<T>
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
    IT: Copy + Into<mint::Point2<T>>,
{
    fn from(coordinate: [IT; 2]) -> Line2<T> {
        Line2::<T>::new(coordinate[0].into(), coordinate[1].into())
    }
}

// [T,T,T,T] -> Line2<T>
impl<T> From<[T; 4]> for Line2<T>
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    fn from(coordinate: [T; 4]) -> Line2<T> {
        Line2::<T>::new(
            [coordinate[0], coordinate[1]].into(),
            [coordinate[2], coordinate[3]].into(),
        )
    }
}

/// A set of 2d linestrings + an aabb
/// Intended to contain related shapes. E.g. outlines of letters with holes
#[derive(PartialEq, Eq, Clone, Hash)]
pub struct LineStringSet2<T>
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    set: Vec<LineString2<T>>,
    aabb: Aabb2<T>,
}

/// A simple 2d AABB
/// If min_max is none the data has not been assigned yet.
#[derive(PartialEq, Eq, Clone, Hash, fmt::Debug)]
pub struct Aabb2<T>
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    min_max: Option<(mint::Point2<T>, mint::Point2<T>)>,
}

#[derive(PartialEq, Eq, Clone, Hash, fmt::Debug)]
pub struct LineString2<T>
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    points: Vec<mint::Point2<T>>,

    /// if connected is set the as_lines() method will add an extra line connecting
    /// the first and last point
    pub connected: bool,
}

impl<T> LineString2<T>
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    pub fn default() -> Self {
        Self {
            points: Vec::<mint::Point2<T>>::new(),
            connected: false,
        }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::<mint::Point2<T>>::with_capacity(capacity),
            connected: false,
        }
    }

    pub fn with_points(mut self, points: Vec<mint::Point2<T>>) -> Self {
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
        I: Iterator<Item = &'a mint::Point2<T>>,
    {
        Self {
            points: iter.into_iter().copied().collect(),
            connected: false,
        }
    }

    /// Returns true if the lines are self intersecting
    /// If number of points < 10 then the intersections are tested using brute force O(n²)
    /// If more than that a sweep-line algorithm is used O(n*log(n))
    pub fn is_self_intersecting(&self) -> Result<bool, LinestringError> {
        if self.points.len() <= 2 {
            Ok(false)
        } else if self.points.len() < 10 {
            let lines = self.as_lines();
            for l0 in lines.iter().enumerate().take(lines.len() - 1) {
                for l1 in lines.iter().enumerate().skip(l0.0 + 1) {
                    if l0.0 == l1.0 {
                        continue;
                    } else if l0.0 + 1 == l1.0 {
                        // consecutive line: l0.0.start <-> l0.0.end_point <-> l2.0.end_point
                        if Line2::intersection_point3(&l0.1.start, &l0.1.end, &l1.1.end)?.is_some()
                        {
                            return Ok(true);
                        }
                    } else if let Some(point) = l0.1.intersection_point(l1.1) {
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
                .with_lines(self.as_lines().into_iter())?
                .compute()?;
            //print!("Lines rv={} [", result.is_empty());
            //for p in self.as_lines().iter() {
            //print!("[{:?},{:?}]-[{:?},{:?}],", p.start.x, p.start.y, p.end.x, p.end.y);
            //    print!("[{:?},{:?},{:?},{:?}],", p.start.x, p.start.y, p.end.x, p.end.y);
            //print!("[{:?},{:?}],", p.x, p.y,);
            //}
            //println!("]");
            Ok(!result.is_empty())
        }
    }

    pub fn points(&self) -> &Vec<mint::Point2<T>> {
        &self.points
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    pub fn as_lines(&self) -> Vec<Line2<T>> {
        if self.points.is_empty() {
            return vec![];
        } else if self.points.len() == 1 {
            return vec![Line2 {
                start: self.points[0],
                end: self.points[0],
            }];
        }
        let iter1 = self.points.iter().skip(1);
        let iter2 = self.points.iter();
        if self.connected && self.points.last() != self.points.first() {
            iter1
                .zip(iter2)
                .map(|(a, b)| Line2 { start: *b, end: *a })
                .chain(
                    Some(Line2 {
                        start: *self.points.last().unwrap(),
                        end: *self.points.first().unwrap(),
                    })
                    .into_iter(),
                )
                .collect()
        } else {
            iter1
                .zip(iter2)
                .map(|(a, b)| Line2 { start: *b, end: *a })
                .collect()
        }
    }

    /// Copy this linestring2 into a linestring3, populating the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    pub fn copy_to_3d(&self, plane: mint_3d::Plane) -> mint_3d::LineString3<T> {
        let mut rv: mint_3d::LineString3<T> = match plane {
            mint_3d::Plane::XY => self
                .points
                .iter()
                .map(|p2d| mint::Point3 {
                    x: p2d.x,
                    y: p2d.y,
                    z: T::zero(),
                })
                .collect(),
            mint_3d::Plane::XZ => self
                .points
                .iter()
                .map(|p2d| mint::Point3 {
                    x: p2d.x,
                    y: T::zero(),
                    z: p2d.y,
                })
                .collect(),
            mint_3d::Plane::ZY => self
                .points
                .iter()
                .map(|p2d| mint::Point3 {
                    x: T::zero(),
                    y: p2d.y,
                    z: p2d.x,
                })
                .collect(),
        };
        rv.connected = self.connected;
        rv
    }

    pub fn push(&mut self, point: mint::Point2<T>) {
        self.points.push(point);
    }

    #[cfg(not(feature = "impl-mint"))]
    pub fn transform(&self, mat: &mint::ColumnMatrix3<T>) -> Self {
        Self {
            points: self
                .points
                .iter()
                .map(|x| mat.transform_point(*x))
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

            let mut rv: Vec<mint::Point2<T>> = Vec::with_capacity(points.len());
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
            let mut rv: Vec<mint::Point2<T>> = Vec::with_capacity(self.points.len());
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

    /// A naïve implementation of Ramer–Douglas–Peucker algorithm
    /// It spawns a lot of Vec, but it seems to work
    fn _simplify(distance_predicate_sq: T, slice: &[mint::Point2<T>]) -> Vec<mint::Point2<T>> {
        //println!("input dist:{:?} slice{:?}", distance_predicate_sq, slice);
        if slice.len() <= 2 {
            return slice[1..].to_vec();
        }
        let start_point = slice.first().unwrap();
        let end_point = slice.last().unwrap();
        let identical_points = point_ulps_eq(&start_point, &end_point);

        let mut max_dist_sq = (-T::one(), 0_usize);

        // find the point with largest distance to start_point<->endpoint line
        for (i, point) in slice.iter().enumerate().take(slice.len() - 1).skip(1) {
            let sq_d = if identical_points {
                distance_to_point_squared(start_point, point)
            } else {
                distance_to_line_squared(start_point, end_point, point)
            };
            //println!("sq_d:{:?}", sq_d);
            if sq_d > max_dist_sq.0 && sq_d > distance_predicate_sq {
                max_dist_sq = (sq_d, i);
            }
        }

        //println!("max_dist_sq: {:?}", max_dist_sq);
        if max_dist_sq.1 == 0 {
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

    /// Simplify using Visvalingam–Whyatt algorithm
    pub fn simplify_vw(&self, _d: T) -> Self {
        unimplemented!();
    }
}

impl<T, IC> std::iter::FromIterator<IC> for LineString2<T>
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
    IC: Into<mint::Point2<T>>,
{
    fn from_iter<I: IntoIterator<Item = IC>>(iter: I) -> Self {
        LineString2 {
            points: iter.into_iter().map(|c| c.into()).collect(),
            connected: false,
        }
    }
}

impl<T> LineStringSet2<T>
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    pub fn default() -> Self {
        Self {
            set: Vec::<LineString2<T>>::new(),
            aabb: Aabb2::default(),
        }
    }

    pub fn set(&self) -> &Vec<LineString2<T>> {
        &self.set
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            set: Vec::<LineString2<T>>::with_capacity(capacity),
            aabb: Aabb2::default(),
        }
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

    pub fn get_aabb(&self) -> &Aabb2<T> {
        &self.aabb
    }

    #[cfg(not(feature = "impl-mint"))]
    pub fn transform(&self, mat: &mint::ColumnMatrix3<T>) -> Self {
        Self {
            aabb: self.aabb.transform(mat),
            set: self.set.iter().map(|x| x.transform(mat)).collect(),
        }
    }

    /// Copy this linestringset2 into a linestringset3, populating the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    /// The empty axis will be set to zero.
    pub fn copy_to_3d(&self, plane: mint_3d::Plane) -> mint_3d::LineStringSet3<T> {
        let mut rv = mint_3d::LineStringSet3::with_capacity(self.set.len());
        for ls in self.set.iter() {
            rv.push(ls.copy_to_3d(plane));
        }
        rv
    }

    /// drains the 'other' container of all shapes and put them into 'self'
    pub fn take_from(&mut self, other: &mut Self) {
        self.aabb.update_aabb(&other.aabb);
        self.set.append(&mut other.set);
    }
}

impl<T, IT> From<[IT; 2]> for Aabb2<T>
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
    IT: Copy + Into<mint::Point2<T>>,
{
    fn from(coordinate: [IT; 2]) -> Aabb2<T> {
        Aabb2 {
            min_max: Some((coordinate[0].into(), coordinate[1].into())),
        }
    }
}

impl<T> From<[T; 4]> for Aabb2<T>
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    fn from(coordinate: [T; 4]) -> Aabb2<T> {
        Aabb2 {
            min_max: Some((
                mint::Point2 {
                    x: coordinate[0],
                    y: coordinate[1],
                },
                mint::Point2 {
                    x: coordinate[2],
                    y: coordinate[3],
                },
            )),
        }
    }
}

impl<T> Aabb2<T>
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    pub fn default() -> Self {
        Self { min_max: None }
    }

    pub fn new(point: &mint::Point2<T>) -> Self {
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

    pub fn update_point(&mut self, point: &mint::Point2<T>) {
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

    pub fn get_high(&self) -> Option<mint::Point2<T>> {
        if let Some((_, _high)) = self.min_max {
            return Some(_high);
        }
        None
    }

    pub fn get_low(&self) -> Option<mint::Point2<T>> {
        if let Some((_low, _)) = self.min_max {
            return Some(_low);
        }
        None
    }

    #[cfg(not(feature = "impl-mint"))]
    pub fn transform(&self, mat: &mint::ColumnMatrix3<T>) -> Self {
        if let Some(min_max) = self.min_max {
            Self {
                min_max: Some((
                    mat.transform_point(min_max.0),
                    mat.transform_point(min_max.1),
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
                return Self::contains_point_(&self_aabb, &other_aabb.0)
                    && Self::contains_point_(&self_aabb, &other_aabb.1);
            }
        }
        false
    }

    /// returns true if this aabb entirely contains/engulfs a line (inclusive)
    #[inline(always)]
    pub fn contains_line(&self, line: &Line2<T>) -> bool {
        if let Some(self_aabb) = self.min_max {
            return Self::contains_point_(&self_aabb, &line.start)
                && Self::contains_point_(&self_aabb, &line.end);
        }
        false
    }

    /// returns true if this aabb contains a point (inclusive)
    #[inline(always)]
    pub fn contains_point(&self, point: &mint::Point2<T>) -> bool {
        if let Some(self_aabb) = self.min_max {
            return Self::contains_point_(&self_aabb, point);
        }
        false
    }

    /// returns true if aabb contains a point (inclusive)
    #[inline(always)]
    fn contains_point_(aabb: &(mint::Point2<T>, mint::Point2<T>), point: &mint::Point2<T>) -> bool {
        aabb.0.x <= point.x && aabb.0.y <= point.y && aabb.1.x >= point.x && aabb.1.y >= point.y
    }
}

/// Get any intersection point between line segment and point.
/// Inspired by https://stackoverflow.com/a/17590923
pub fn intersect_line_point<T>(line: &Line2<T>, point: &mint::Point2<T>) -> Option<Intersection<T>>
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    // take care of end point equality
    if ulps_eq(&line.start.x, &point.x) && ulps_eq(&line.start.y, &point.y) {
        return Some(Intersection::Intersection(*point));
    }
    if ulps_eq(&line.end.x, &point.x) && ulps_eq(&line.end.y, &point.y) {
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
    if ulps_eq(&ab, &(ap + pb)) {
        return Some(Intersection::Intersection(*point));
    }
    None
}

#[allow(dead_code)]
pub enum Intersection<T>
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    // Normal one point intersection
    Intersection(mint::Point2<T>),
    // Collinear overlapping
    OverLap(Line2<T>),
}

impl<T> Intersection<T>
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    /// return a single, simple intersection point
    pub fn single(&self) -> mint::Point2<T> {
        match self {
            Self::OverLap(a) => a.start,
            Self::Intersection(a) => *a,
        }
    }
}

impl<T> fmt::Debug for Intersection<T>
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
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
    point: &mint::Point2<T>,
    vector: &mint::Vector2<T>,
    scale: T,
) -> mint::Point2<T>
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    mint::Point2 {
        x: point.x + scale * vector.x,
        y: point.y + scale * vector.y,
    }
}

#[inline(always)]
/// Divides a 'vector' by 'b'. Obviously, don't feed this with 'b' == 0
fn div<T>(a: &mint::Vector2<T>, b: T) -> mint::Vector2<T>
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    mint::Vector2 {
        x: a.x / b,
        y: a.y / b,
    }
}

#[inline(always)]
/// subtracts point b from point a resulting in a vector
fn sub<T>(a: &mint::Point2<T>, b: &mint::Point2<T>) -> mint::Vector2<T>
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    mint::Vector2 {
        x: a.x - b.x,
        y: a.y - b.y,
    }
}

#[inline(always)]
/// from https://stackoverflow.com/a/565282 :
///  "Define the 2-dimensional vector cross product v × w to be vx wy − vy wx."
/// This function returns the z component of v × w (if we pretend v and w are two dimensional)
fn cross_z<T>(v: &mint::Vector2<T>, w: &mint::Vector2<T>) -> T
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    v.x * w.y - v.y * w.x
}

#[inline(always)]
/// The distance between the line a->b to the point p is the same as
/// distance = |(a-p)×(a-b)|/|a-b|
/// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Another_vector_formulation
/// Make sure to *not* call this function with a-b==0
/// This function returns the distance²
pub fn distance_to_line_squared<T>(
    a: &mint::Point2<T>,
    b: &mint::Point2<T>,
    p: &mint::Point2<T>,
) -> T
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    let a_sub_b = sub(a, b);
    let a_sub_p = sub(a, p);
    let a_sub_p_cross_a_sub_b = cross_z(&a_sub_p, &a_sub_b);
    (a_sub_p_cross_a_sub_b * a_sub_p_cross_a_sub_b)
        / (a_sub_b.x * a_sub_b.x + a_sub_b.y * a_sub_b.y)
}

#[inline(always)]
/// The distance² between the two points
pub fn distance_to_point_squared<T>(a: &mint::Point2<T>, b: &mint::Point2<T>) -> T
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    let v = sub(a, b);
    v.x * v.x + v.y * v.y
}

#[inline(always)]
/// calculate the dot product of two vectors
fn dot<T>(a: &mint::Vector2<T>, b: &mint::Vector2<T>) -> T
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    a.x * b.x + a.y * b.y
}

#[inline(always)]
pub fn point_ulps_eq<T>(a: &mint::Point2<T>, b: &mint::Point2<T>) -> bool
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    ulps_eq(&a.x, &b.x) && ulps_eq(&a.y, &b.y)
}

#[inline(always)]
pub fn ulps_eq<T>(a: &T, b: &T) -> bool
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    T::ulps_eq(a, b, T::default_epsilon(), T::default_max_ulps())
}
