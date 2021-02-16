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
                let dx = high_bound.x - low_bound.x;
                let dy = high_bound.y - low_bound.y;
                let dz = high_bound.z - low_bound.z;
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
    pub start: mint::Point2<T>,
    pub end: mint::Point2<T>,
}

impl<T> Line2<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
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
        [self.start.x, self.start.y, self.end.x, self.end.y]
    }
}

impl<T, IT> From<[IT; 2]> for Line2<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
    IT: Copy + Into<mint::Point2<T>>,
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
    min_max: Option<(mint::Point2<T>, mint::Point2<T>)>,
}

#[derive(PartialEq, Eq, Clone, Hash, fmt::Debug)]
pub struct LineString2<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    points: Vec<mint::Point2<T>>,

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
    pub start: mint::Point3<T>,
    pub end: mint::Point3<T>,
}

impl<T> Line3<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    pub fn new(start: mint::Point3<T>, end: mint::Point3<T>) -> Self {
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
            self.start.x,
            self.start.y,
            self.start.z,
            self.end.x,
            self.end.y,
            self.end.z,
        ]
    }
}

impl<T, IT> From<[IT; 2]> for Line3<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
    IT: Copy + Into<mint::Point3<T>>,
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
    points: Vec<mint::Point3<T>>,

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
    min_max: Option<(mint::Point3<T>, mint::Point3<T>)>,
}

impl<T> LineString2<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
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
        let mut rv: LineString3<T> = self
            .points
            .iter()
            .map(|p2d| match plane {
                Plane::XY => mint::Point3 {
                    x: p2d.x,
                    y: p2d.y,
                    z: T::zero(),
                },
                Plane::XZ => mint::Point3 {
                    x: p2d.x,
                    y: T::zero(),
                    z: p2d.y,
                },
                Plane::ZY => mint::Point3 {
                    x: T::zero(),
                    y: p2d.y,
                    z: p2d.x,
                },
            })
            .collect();
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
    pub fn simplify(&self, _d: T) -> Self {
        unimplemented!();
    }

    /// Simplify using Visvalingam–Whyatt algorithm
    pub fn simplify_vw(&self, _d: T) -> Self {
        unimplemented!();
    }
}

impl<T, IC> std::iter::FromIterator<IC> for LineString2<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
    IC: Into<mint::Point2<T>>,
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
            points: Vec::<mint::Point3<T>>::new(),
            connected: false,
        }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::<mint::Point3<T>>::with_capacity(capacity),
            connected: false,
        }
    }

    /// Copy this linestring3 into a linestring2, keeping the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    pub fn copy_to_2d(&self, plane: Plane) -> LineString2<T> {
        let mut rv: LineString2<T> = self
            .points
            .iter()
            .map(|p3d| match plane {
                Plane::XY => mint::Point2 { x: p3d.x, y: p3d.y },
                Plane::XZ => mint::Point2 { x: p3d.x, y: p3d.z },
                Plane::ZY => mint::Point2 { x: p3d.z, y: p3d.y },
            })
            .collect();
        rv.connected = self.connected;
        rv
    }

    pub fn points(&self) -> &Vec<mint::Point3<T>> {
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

    pub fn push(&mut self, point: mint::Point3<T>) {
        self.points.push(point);
    }

    #[cfg(not(feature = "impl-mint"))]
    pub fn transform(&self, mat: &mint::ColumnMatrix4<T>) -> Self {
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

impl<T, IC: Into<mint::Point3<T>>> std::iter::FromIterator<IC> for LineString3<T>
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

    #[cfg(not(feature = "impl-mint"))]
    pub fn transform(&self, mat: &mint::ColumnMatrix4<T>) -> Self {
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
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
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
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
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

impl<T, IT> From<[IT; 2]> for Aabb3<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
    IT: Copy + Into<mint::Point3<T>>,
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
                mint::Point3 {
                    x: coordinate[0],
                    y: coordinate[1],
                    z: coordinate[2],
                },
                mint::Point3 {
                    x: coordinate[3],
                    y: coordinate[4],
                    z: coordinate[5],
                },
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

    pub fn update_point(&mut self, point: &mint::Point3<T>) {
        if self.min_max.is_none() {
            self.min_max = Some((*point, *point));
        }
        let (mut aabb_min, mut aabb_max) = self.min_max.take().unwrap();

        if point.x < aabb_min.x {
            aabb_min.x = point.x;
        }
        if point.y < aabb_min.y {
            aabb_min.y = point.y;
        }
        if point.z < aabb_min.z {
            aabb_min.z = point.z;
        }
        if point.x > aabb_max.x {
            aabb_max.x = point.x;
        }
        if point.y > aabb_max.y {
            aabb_max.y = point.y;
        }
        if point.z > aabb_max.z {
            aabb_max.z = point.z;
        }
        self.min_max = Some((aabb_min, aabb_max));
    }

    pub fn get_high(&self) -> Option<mint::Point3<T>> {
        if let Some((_, _high)) = self.min_max {
            return Some(_high);
        }
        None
    }

    pub fn get_low(&self) -> Option<mint::Point3<T>> {
        if let Some((_low, _)) = self.min_max {
            return Some(_low);
        }
        None
    }

    #[cfg(not(feature = "impl-mint"))]
    pub fn transform(&self, mat: &mint::ColumnMatrix4<T>) -> Self {
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
    pub fn contains_point(&self, point: &mint::Point3<T>) -> bool {
        if let Some(self_aabb) = self.min_max {
            return Self::contains_point_(&self_aabb, point);
        }
        false
    }

    /// returns true if aabb contains a point (inclusive)
    #[inline(always)]
    fn contains_point_(aabb: &(mint::Point3<T>, mint::Point3<T>), point: &mint::Point3<T>) -> bool {
        aabb.0.x <= point.x
            && aabb.0.y <= point.y
            && aabb.0.z <= point.z
            && aabb.1.x >= point.x
            && aabb.1.y >= point.y
            && aabb.1.z >= point.z
    }
}

/// Get any intersection point between line segment and point.
/// Inspired by https://stackoverflow.com/a/17590923
pub fn intersect_line_point<T>(line: &Line2<T>, point: &mint::Point2<T>) -> Option<Intersection<T>>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
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
    Intersection(mint::Point2<T>),
    // Collinear overlapping
    OverLap(Line2<T>),
}

impl<T> Intersection<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
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
pub fn scale_to_coordinate<T>(
    point: &mint::Point2<T>,
    vector: &mint::Vector2<T>,
    scale: T,
) -> mint::Point2<T>
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
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
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
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
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    mint::Vector2 {
        x: a.x - b.x,
        y: a.y - b.y,
    }
}

#[inline(always)]
/// from https://stackoverflow.com/a/565282 :
///  "Define the 2-dimensional vector cross product v × w to be vx wy − vy wx."
/// This function returns the z component of v × w
fn cross_z<T>(v: &mint::Vector2<T>, w: &mint::Vector2<T>) -> T
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    v.x * w.y - v.y * w.x
}

#[inline(always)]
/// calculate the dot product of two vectors
fn dot<T>(a: &mint::Vector2<T>, b: &mint::Vector2<T>) -> T
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    a.x * b.x + a.y * b.y
}

#[inline(always)]
pub fn ulps_eq_c<T>(a: &mint::Point2<T>, b: &mint::Point2<T>) -> bool
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    ulps_eq(&a.x, &b.x) && ulps_eq(&a.y, &b.y)
}

#[inline(always)]
pub fn ulps_eq<T>(a: &T, b: &T) -> bool
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    T::ulps_eq(a, b, T::default_epsilon(), T::default_max_ulps())
}
