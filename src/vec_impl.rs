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

use num_traits::Float;
use std::fmt;

/// Axis aligned planes, used to describe how imported 'flat' data is arranged in space
#[derive(fmt::Debug, Copy, Clone)]
pub enum Plane {
    XY,
    XZ,
    ZY,
}

impl Plane {
    /// Try to figure out what axes defines the plane.
    /// If the AABB delta of one axis (a) is virtually nothing compared to
    /// the widest axis (b) while the third axis (c) is comparable to (b)
    /// by some fraction, we assume that that (a) isn't part of the plane.
    ///
    /// It's not possible to compare to zero exactly because blender
    /// leaves some decimal in coordinates that's suppose to be zero.
    pub fn get_plane<T>(aabb: &Aabb3<T>) -> Option<Plane>
    where
        T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
    {
        if let Some(low_bound) = aabb.get_low() {
            if let Some(high_bound) = aabb.get_high() {
                let dx = high_bound[0] - low_bound[0];
                let dy = high_bound[1] - low_bound[1];
                let dz = high_bound[2] - low_bound[2];
                let max_delta = Float::max(Float::max(dx, dy), dz);

                let dx = T::zero().ulps_eq(
                    &(dx / max_delta),
                    T::default_epsilon(),
                    T::default_max_ulps(),
                );
                let dy = T::zero().ulps_eq(
                    &&(dy / max_delta),
                    T::default_epsilon(),
                    T::default_max_ulps(),
                );
                let dz = T::zero().ulps_eq(
                    &&(dz / max_delta),
                    T::default_epsilon(),
                    T::default_max_ulps(),
                );

                if dx && !dy && !dz {
                    return Some(Plane::XY);
                }
                if dy && !dx && !dz {
                    return Some(Plane::XZ);
                }
                if dz && !dx && !dy {
                    return Some(Plane::ZY);
                }
            }
        }
        None
    }
}

/// A 2d line
#[derive(PartialEq, Eq, Copy, Clone, Hash, fmt::Debug)]
pub struct Line2<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    pub start: [T; 2],
    pub end: [T; 2],
}

impl<T> Line2<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    pub fn new(start: [T; 2], end: [T; 2]) -> Self {
        Self { start, end }
    }

    /// Get any intersection point between line segments.
    /// Note that this function always detects endpoint-to-endpoint intersections.
    /// Most of this is from https://stackoverflow.com/a/565282
    #[allow(clippy::many_single_char_names)]
    pub fn intersection_point(&self, other: &Self) -> Option<Intersection<T>>
    where
        T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
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
            let one_is_a_point = ulps_eq_c(&self.start, &self.end);
            let other_is_a_point = ulps_eq_c(&other.start, &other.end);
            if one_is_a_point || other_is_a_point {
                if one_is_a_point && other_is_a_point && ulps_eq_c(&self.start, &other.start) {
                    return Some(Intersection::Intersection(self.start));
                }
                return if one_is_a_point {
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
}

#[allow(clippy::from_over_into)]
impl<T> Into<[T; 4]> for Line2<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    fn into(self) -> [T; 4] {
        [self.start[0], self.start[1], self.end[0], self.end[1]]
    }
}

impl<T, IT> From<[IT; 2]> for Line2<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
    IT: Copy + Into<[T; 2]>,
{
    fn from(coordinate: [IT; 2]) -> Line2<T> {
        Line2::<T>::new(coordinate[0].into(), coordinate[1].into())
    }
}

impl<T> From<[T; 4]> for Line2<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    fn from(coordinate: [T; 4]) -> Line2<T> {
        Line2::<T>::new(
            [coordinate[0], coordinate[1]],
            [coordinate[2], coordinate[3]],
        )
    }
}

/// A set of 2d linestrings + an aabb
/// Intended to contain related shapes. E.g. outlines of letters with holes
#[derive(PartialEq, Eq, Clone, Hash)]
pub struct LineStringSet2<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    set: Vec<LineString2<T>>,
    aabb: Aabb2<T>,
}

/// A simple 2d AABB
/// If min_max is none the data has not been assigned yet.
#[derive(PartialEq, Eq, Clone, Hash, fmt::Debug)]
pub struct Aabb2<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    min_max: Option<([T; 2], [T; 2])>,
}

#[derive(PartialEq, Eq, Clone, Hash, fmt::Debug)]
pub struct LineString2<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    points: Vec<[T; 2]>,

    /// if connected is set the as_lines() method will add an extra line connecting
    /// the first and last point
    pub connected: bool,
}

/// A 3d line
#[derive(PartialEq, Eq, Copy, Clone, Hash, fmt::Debug)]
pub struct Line3<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    pub start: [T; 3],
    pub end: [T; 3],
}

impl<T> Line3<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    pub fn new(start: [T; 3], end: [T; 3]) -> Self {
        Self { start, end }
    }
}

#[allow(clippy::from_over_into)]
impl<T> Into<[T; 6]> for Line3<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    fn into(self) -> [T; 6] {
        [
            self.start[0],
            self.start[1],
            self.start[2],
            self.end[0],
            self.end[1],
            self.end[2],
        ]
    }
}

impl<T, IT> From<[IT; 2]> for Line3<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
    IT: Copy + Into<[T; 3]>,
{
    fn from(pos: [IT; 2]) -> Line3<T> {
        Line3::<T>::new(pos[0].into(), pos[1].into())
    }
}

#[derive(PartialEq, Eq, Clone, Hash, fmt::Debug)]
pub struct LineString3<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    points: Vec<[T; 3]>,

    /// if connected is set the as_lines() method will add an extra line connecting
    /// the first and last point
    pub connected: bool,
}

/// A set of linestrings + an aabb
/// Intended to contain related 3d shapes. E.g. outlines of letters with holes
#[derive(PartialEq, Eq, Clone, Hash)]
pub struct LineStringSet3<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    pub set: Vec<LineString3<T>>,
    pub aabb: Aabb3<T>,
}

/// A simple 3d AABB
/// If min_max is none the data has not been assigned yet.
#[derive(PartialEq, Eq, Copy, Clone, Hash, fmt::Debug)]
pub struct Aabb3<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    min_max: Option<([T; 3], [T; 3])>,
}

impl<T> LineString2<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    pub fn default() -> Self {
        Self {
            points: Vec::<[T; 2]>::new(),
            connected: false,
        }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::<[T; 2]>::with_capacity(capacity),
            connected: false,
        }
    }

    /// Copies the points of the iterator into the LineString2
    /// from_iter is already claimed for into() objects.
    pub fn with_iter<'a, I>(iter: I) -> Self
    where
        T: 'a,
        I: Iterator<Item = &'a [T; 2]>,
    {
        Self {
            points: iter.into_iter().copied().collect(),
            connected: false,
        }
    }

    pub fn points(&self) -> &Vec<[T; 2]> {
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
                start: *self.points.first().unwrap(),
                end: *self.points.first().unwrap(),
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
    pub fn copy_to_3d(&self, plane: Plane) -> LineString3<T> {
        let mut rv: LineString3<T> = match plane {
            Plane::XY => self
                .points
                .iter()
                .map(|p2d| [p2d[0], p2d[1], T::zero()])
                .collect(),
            Plane::XZ => self
                .points
                .iter()
                .map(|p2d| [p2d[0], T::zero(), p2d[1]])
                .collect(),
            Plane::ZY => self
                .points
                .iter()
                .map(|p2d| [T::zero(), p2d[1], p2d[0]])
                .collect(),
        };
        rv.connected = self.connected;
        rv
    }

    pub fn push(&mut self, point: [T; 2]) {
        self.points.push(point);
    }

    #[cfg(not(feature = "impl-vec"))]
    pub fn transform(&self, mat: &cgmath::Matrix3<T>) -> Self {
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
            let start_point = self.points.first().unwrap();
            let mut max_dist_sq = (-T::one(), 0_usize);

            // find the point with largest distance from the start_point
            for (i, point) in self.points.iter().enumerate().skip(1) {
                let sq_d = distance_to_point2_squared(start_point, point);
                //println!("sq_d:{:?} i:{:?} s:{:?} p:{:?}", sq_d, i,start_point, point);
                if sq_d > max_dist_sq.0 {
                    max_dist_sq = (sq_d, i);
                }
            }
            //println!("max_dist_sq: {:?}", max_dist_sq);
            let mut rv: Vec<[T; 2]> = Vec::with_capacity(self.points.len());
            rv.push(*self.points.first().unwrap());
            rv.append(&mut Self::_simplify(
                distance_predicate * distance_predicate,
                &self.points.as_slice()[..max_dist_sq.1 + 1],
            ));
            {
                let mut tmp: Vec<[T; 2]> = self.points.as_slice()[max_dist_sq.1..]
                    .iter()
                    .copied()
                    .collect();
                // add start_point as end point
                tmp.push(*start_point);
                rv.append(&mut Self::_simplify(
                    distance_predicate * distance_predicate,
                    &tmp.as_slice()[..],
                ));
            }
            // remove the 'extra' end point we just added.
            let _ = rv.remove(rv.len() - 1);
            Self {
                points: rv,
                connected: true,
            }
        } else {
            let mut rv: Vec<[T; 2]> = Vec::with_capacity(self.points.len());
            // _simplify() always omits the the first point, so we have to add that
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
    /// TODO: make sure this isn't called with endpoint==startpoint!!
    fn _simplify(distance_predicate_sq: T, slice: &[[T; 2]]) -> Vec<[T; 2]> {
        //println!("input dist:{:?} slice{:?}", distance_predicate_sq, slice);
        if slice.len() <= 2 {
            return slice[1..].to_vec();
        }
        let start_point = slice.first().unwrap();
        let end_point = slice.last().unwrap();
        let mut max_dist_sq = (-T::one(), 0_usize);

        // find the point with largest distance to start_point<->endpoint line
        for (i, point) in slice.iter().enumerate().take(slice.len() - 1).skip(1) {
            let sq_d = distance_to_line2_squared(start_point, end_point, point);
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
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
    IC: Into<[T; 2]>,
{
    fn from_iter<I: IntoIterator<Item = IC>>(iter: I) -> Self {
        LineString2 {
            points: iter.into_iter().map(|c| c.into()).collect(),
            connected: false,
        }
    }
}

impl<T> LineString3<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    pub fn default() -> Self {
        Self {
            points: Vec::<[T; 3]>::new(),
            connected: false,
        }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::<[T; 3]>::with_capacity(capacity),
            connected: false,
        }
    }

    /// Copies the points of the iterator into the LineString2
    /// from_iter is already claimed for into() objects.
    pub fn with_iter<'a, I>(iter: I) -> Self
    where
        T: 'a,
        I: Iterator<Item = &'a [T; 3]>,
    {
        Self {
            points: iter.into_iter().copied().collect(),
            connected: false,
        }
    }

    /// Copy this linestring3 into a linestring2, keeping the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    pub fn copy_to_2d(&self, plane: Plane) -> LineString2<T> {
        let mut rv: LineString2<T> = match plane {
            Plane::XY => self.points.iter().map(|p3d| [p3d[0], p3d[1]]).collect(),
            Plane::XZ => self.points.iter().map(|p3d| [p3d[0], p3d[2]]).collect(),
            Plane::ZY => self.points.iter().map(|p3d| [p3d[2], p3d[1]]).collect(),
        };
        rv.connected = self.connected;
        rv
    }

    pub fn points(&self) -> &Vec<[T; 3]> {
        &self.points
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    pub fn as_lines(&self) -> Vec<Line3<T>> {
        if self.points.is_empty() {
            return vec![];
        } else if self.points.len() == 1 {
            return vec![Line3 {
                start: *self.points.first().unwrap(),
                end: *self.points.first().unwrap(),
            }];
        }
        let iter1 = self.points.iter().skip(1);
        let iter2 = self.points.iter();
        if self.connected && self.points.last() != self.points.first() {
            iter1
                .zip(iter2)
                .map(|(a, b)| Line3 { start: *b, end: *a })
                .chain(
                    Some(Line3 {
                        start: *self.points.last().unwrap(),
                        end: *self.points.first().unwrap(),
                    })
                    .into_iter(),
                )
                .collect()
        } else {
            iter1
                .zip(iter2)
                .map(|(a, b)| Line3 { start: *b, end: *a })
                .collect()
        }
    }

    pub fn push(&mut self, point: [T; 3]) {
        self.points.push(point);
    }

    #[cfg(not(feature = "impl-vec"))]
    pub fn transform(&self, mat: &cgmath::Matrix4<T>) -> Self {
        Self {
            points: self
                .points
                .iter()
                .map(|x| mat.transform_point(*x))
                .collect(),
            connected: self.connected,
        }
    }

    /// Simplify using Ramer–Douglas–Peucker algorithm adapted for 3d
    pub fn simplify(&self, _d: T) -> Self {
        unimplemented!();
    }

    /// Simplify using Visvalingam–Whyatt algorithm adapted for 3d
    pub fn simplify_vw(&self, _d: T) -> Self {
        unimplemented!();
    }
}

impl<T, IC: Into<[T; 3]>> std::iter::FromIterator<IC> for LineString3<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    fn from_iter<I: IntoIterator<Item = IC>>(iter: I) -> Self {
        LineString3 {
            points: iter.into_iter().map(|c| c.into()).collect(),
            connected: false,
        }
    }
}

impl<T> LineStringSet2<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
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

    #[cfg(not(feature = "impl-vec"))]
    pub fn transform(&self, mat: &cgmath::Matrix3<T>) -> Self {
        Self {
            aabb: self.aabb.transform(mat),
            set: self.set.iter().map(|x| x.transform(mat)).collect(),
        }
    }

    /// Copy this linestringset2 into a linestringset3, populating the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    /// The empty axis will be set to zero.
    pub fn copy_to_3d(&self, plane: Plane) -> LineStringSet3<T> {
        let mut rv = LineStringSet3::with_capacity(self.set.len());
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

impl<T> LineStringSet3<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    pub fn default() -> Self {
        Self {
            set: Vec::<LineString3<T>>::new(),
            aabb: Aabb3::default(),
        }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            set: Vec::<LineString3<T>>::with_capacity(capacity),
            aabb: Aabb3::default(),
        }
    }

    pub fn set(&self) -> &Vec<LineString3<T>> {
        &self.set
    }

    pub fn is_empty(&self) -> bool {
        self.set.is_empty()
    }

    pub fn push(&mut self, ls: LineString3<T>) {
        if !ls.is_empty() {
            self.set.push(ls);

            for ls in self.set.last().unwrap().points.iter() {
                self.aabb.update_point(ls);
            }
        }
    }

    pub fn get_aabb(&self) -> &Aabb3<T> {
        &self.aabb
    }

    #[cfg(not(feature = "impl-vec"))]
    pub fn transform(&self, mat: &cgmath::Matrix4<T>) -> Self {
        Self {
            set: self.set.iter().map(|x| x.transform(mat)).collect(),
            aabb: self.aabb.transform(mat),
        }
    }

    /// Copy this linestringset3 into a linestringset2, populating the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    pub fn copy_to_2d(&self, plane: Plane) -> LineStringSet2<T> {
        let mut rv = LineStringSet2::with_capacity(self.set.len());
        for ls in self.set.iter() {
            rv.push(ls.copy_to_2d(plane));
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
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
    IT: Copy + Into<[T; 2]>,
{
    fn from(coordinate: [IT; 2]) -> Aabb2<T> {
        Aabb2 {
            min_max: Some((coordinate[0].into(), coordinate[1].into())),
        }
    }
}

impl<T> From<[T; 4]> for Aabb2<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    fn from(coordinate: [T; 4]) -> Aabb2<T> {
        Aabb2 {
            min_max: Some((
                [coordinate[0], coordinate[1]],
                [coordinate[2], coordinate[3]],
            )),
        }
    }
}

impl<T> Aabb2<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    pub fn default() -> Self {
        Self { min_max: None }
    }

    pub fn new(point: &[T; 2]) -> Self {
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

    pub fn update_point(&mut self, point: &[T; 2]) {
        if self.min_max.is_none() {
            self.min_max = Some((*point, *point));
            return;
        }
        let (mut aabb_min, mut aabb_max) = self.min_max.take().unwrap();

        if point[0] < aabb_min[0] {
            aabb_min[0] = point[0];
        }
        if point[1] < aabb_min[1] {
            aabb_min[1] = point[1];
        }
        if point[0] > aabb_max[0] {
            aabb_max[0] = point[0];
        }
        if point[1] > aabb_max[1] {
            aabb_max[1] = point[1];
        }
        self.min_max = Some((aabb_min, aabb_max));
    }

    pub fn get_high(&self) -> Option<[T; 2]> {
        if let Some((_, _high)) = self.min_max {
            return Some(_high);
        }
        None
    }

    pub fn get_low(&self) -> Option<[T; 2]> {
        if let Some((_low, _)) = self.min_max {
            return Some(_low);
        }
        None
    }

    #[cfg(not(feature = "impl-vec"))]
    pub fn transform(&self, mat: &cgmath::Matrix3<T>) -> Self {
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
    pub fn contains_point(&self, point: &[T; 2]) -> bool {
        if let Some(self_aabb) = self.min_max {
            return Self::contains_point_(&self_aabb, point);
        }
        false
    }

    /// returns true if aabb contains a point (inclusive)
    #[inline(always)]
    fn contains_point_(aabb: &([T; 2], [T; 2]), point: &[T; 2]) -> bool {
        aabb.0[0] <= point[0]
            && aabb.0[1] <= point[1]
            && aabb.1[0] >= point[0]
            && aabb.1[1] >= point[1]
    }
}

impl<T, IT> From<[IT; 2]> for Aabb3<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
    IT: Copy + Into<[T; 3]>,
{
    fn from(coordinate: [IT; 2]) -> Aabb3<T> {
        Aabb3 {
            min_max: Some((coordinate[0].into(), coordinate[1].into())),
        }
    }
}

impl<T> From<[T; 6]> for Aabb3<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    fn from(coordinate: [T; 6]) -> Aabb3<T> {
        Aabb3 {
            min_max: Some((
                [coordinate[0], coordinate[1], coordinate[2]],
                [coordinate[3], coordinate[4], coordinate[5]],
            )),
        }
    }
}

impl<T> Aabb3<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    pub fn default() -> Self {
        Self { min_max: None }
    }

    pub fn update_aabb(&mut self, aabb: &Aabb3<T>) {
        if let Some((min, max)) = &aabb.min_max {
            self.update_point(min);
            self.update_point(max);
        }
    }

    pub fn update_point(&mut self, point: &[T; 3]) {
        if self.min_max.is_none() {
            self.min_max = Some((*point, *point));
            return;
        }
        let (mut aabb_min, mut aabb_max) = self.min_max.take().unwrap();

        if point[0] < aabb_min[0] {
            aabb_min[0] = point[0];
        }
        if point[1] < aabb_min[1] {
            aabb_min[1] = point[1];
        }
        if point[2] < aabb_min[2] {
            aabb_min[2] = point[2];
        }
        if point[0] > aabb_max[0] {
            aabb_max[0] = point[0];
        }
        if point[1] > aabb_max[1] {
            aabb_max[1] = point[1];
        }
        if point[2] > aabb_max[2] {
            aabb_max[2] = point[2];
        }
        self.min_max = Some((aabb_min, aabb_max));
    }

    pub fn get_high(&self) -> Option<[T; 3]> {
        if let Some((_, _high)) = self.min_max {
            return Some(_high);
        }
        None
    }

    pub fn get_low(&self) -> Option<[T; 3]> {
        if let Some((_low, _)) = self.min_max {
            return Some(_low);
        }
        None
    }

    #[cfg(not(feature = "impl-vec"))]
    pub fn transform(&self, mat: &cgmath::Matrix4<T>) -> Self {
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
    pub fn contains_aabb(&self, other: &Self) -> bool {
        if let Some(self_aabb) = other.min_max {
            if let Some(o_aabb) = other.min_max {
                return Self::contains_point_(&self_aabb, &o_aabb.0)
                    && Self::contains_point_(&self_aabb, &o_aabb.1);
            }
        }
        false
    }

    /// returns true if this aabb entirely contains/engulfs a line (inclusive)
    #[inline(always)]
    pub fn contains_line(&self, line: &Line3<T>) -> bool {
        if let Some(self_aabb) = self.min_max {
            return Self::contains_point_(&self_aabb, &line.start)
                && Self::contains_point_(&self_aabb, &line.end);
        }
        false
    }

    /// returns true if this aabb contains a point (inclusive)
    #[inline(always)]
    pub fn contains_point(&self, point: &[T; 3]) -> bool {
        if let Some(self_aabb) = self.min_max {
            return Self::contains_point_(&self_aabb, point);
        }
        false
    }

    /// returns true if aabb contains a point (inclusive)
    #[inline(always)]
    fn contains_point_(aabb: &([T; 3], [T; 3]), point: &[T; 3]) -> bool {
        aabb.0[0] <= point[0]
            && aabb.0[1] <= point[1]
            && aabb.0[2] <= point[2]
            && aabb.1[0] >= point[0]
            && aabb.1[1] >= point[1]
            && aabb.1[2] >= point[2]
    }
}

/// Get any intersection point between line segment and point.
/// Inspired by https://stackoverflow.com/a/17590923
pub fn intersect_line_point<T>(line: &Line2<T>, point: &[T; 2]) -> Option<Intersection<T>>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    // take care of end point equality
    if ulps_eq(&line.start[0], &point[0]) && ulps_eq(&line.start[1], &point[1]) {
        return Some(Intersection::Intersection(*point));
    }
    if ulps_eq(&line.end[0], &point[0]) && ulps_eq(&line.end[1], &point[1]) {
        return Some(Intersection::Intersection(*point));
    }

    let x1 = line.start[0];
    let x2 = line.end[0];
    let y1 = line.start[1];
    let y2 = line.end[1];
    let x = point[0];
    let y = point[1];

    let ab = Float::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    let ap = Float::sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
    let pb = Float::sqrt((x2 - x) * (x2 - x) + (y2 - y) * (y2 - y));

    #[cfg(feature = "console_trace")]
    println!("ab={}, ap={}, pb={}, ap+pb={}", ab, ap, pb, ap + pb);
    if ulps_eq(&ab, &(ap + pb)) {
        return Some(Intersection::Intersection(*point));
    }
    None
}

#[allow(dead_code)]
pub enum Intersection<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    // Normal one point intersection
    Intersection([T; 2]),
    // Collinear overlapping
    OverLap(Line2<T>),
}

impl<T> Intersection<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    /// return a single, simple intersection point
    pub fn single(&self) -> [T; 2] {
        match self {
            Self::OverLap(a) => a.start,
            Self::Intersection(a) => *a,
        }
    }
}

impl<T> fmt::Debug for Intersection<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::OverLap(a) => a.fmt(f),
            Self::Intersection(a) => a.fmt(f),
        }
    }
}

#[inline(always)]
pub fn scale_to_coordinate<T>(point: &[T; 2], vector: &[T; 2], scale: T) -> [T; 2]
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    [point[0] + scale * vector[0], point[1] + scale * vector[1]]
}

#[inline(always)]
/// Divides a 'vector' by 'b'. Obviously, don't feed this with 'b' == 0
fn div<T>(a: &[T; 2], b: T) -> [T; 2]
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    [a[0] / b, a[1] / b]
}

#[inline(always)]
/// subtracts point b from point a resulting in a vector
fn sub<T>(a: &[T; 2], b: &[T; 2]) -> [T; 2]
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    [a[0] - b[0], a[1] - b[1]]
}

#[inline(always)]
/// from https://stackoverflow.com/a/565282 :
///  "Define the 2-dimensional vector cross product v × w to be vx wy − vy wx."
/// This function returns the z component of v × w (if we pretend v and w are two dimensional)
fn cross_z<T>(v: &[T; 2], w: &[T; 2]) -> T
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    v[0] * w[1] - v[1] * w[0]
}

#[inline(always)]
/// The distance between the line a->b to the point p is the same as
/// distance = |(a-p)×(a-b)|/|a-b|
/// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Another_vector_formulation
/// Make sure to *not* call this function with a-b==0
pub fn distance_to_line2_squared<T>(a: &[T; 2], b: &[T; 2], p: &[T; 2]) -> T
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    let a_sub_b = sub(a, b);
    let a_sub_p = sub(a, p);
    let a_sub_p_cross_a_sub_b = cross_z(&a_sub_p, &a_sub_b);
    (a_sub_p_cross_a_sub_b * a_sub_p_cross_a_sub_b)
        / (a_sub_b[0] * a_sub_b[0] + a_sub_b[1] * a_sub_b[1])
}

#[inline(always)]
/// The distance between the two points
pub fn distance_to_point2_squared<T>(a: &[T; 2], b: &[T; 2]) -> T
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    let v = sub(a, b);
    v[0] * v[0] + v[1] * v[1]
}

#[inline(always)]
/// calculate the dot product of two vectors
fn dot<T>(a: &[T; 2], b: &[T; 2]) -> T
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    a[0] * b[0] + a[1] * b[1]
}

#[inline(always)]
pub fn ulps_eq_c<T>(a: &[T; 2], b: &[T; 2]) -> bool
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    ulps_eq(&a[0], &b[0]) && ulps_eq(&a[1], &b[1])
}

#[inline(always)]
pub fn ulps_eq<T>(a: &T, b: &T) -> bool
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    T::ulps_eq(a, b, T::default_epsilon(), T::default_max_ulps())
}
