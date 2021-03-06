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

use super::vector_2d;
use crate::LinestringError;
use approx::ulps_eq;
//#[cfg(feature = "vecmath")]
//use vecmath::Matrix4;
use itertools::Itertools;
use std::collections;
use std::fmt;
use std::fs;
use std::hash::Hash;
use std::io;
use std::io::Write;
use std::path;

/// Placeholder for different 3d shapes
pub enum Shape3d<T>
where
    T: num_traits::Float
        + std::fmt::Debug
        + approx::AbsDiffEq
        + approx::UlpsEq
        + Sync
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>,
{
    Line(Line3<T>),
    Linestring(LineString3<T>),
    ParabolicArc(crate::vector_2d::VoronoiParabolicArc<T>),
}

/// Axis aligned planes, used to describe how imported 'flat' data is arranged in space
#[allow(clippy::upper_case_acronyms)]
#[derive(fmt::Debug, Copy, Clone)]
pub enum Plane {
    XY,
    XZ,
    ZY,
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
    pub fn get_plane<T>(aabb: &Aabb3<T>) -> Option<Self>
    where
        T: num_traits::Float
            + std::fmt::Debug
            + approx::AbsDiffEq
            + approx::UlpsEq
            + Sync
            + approx::AbsDiffEq<Epsilon = T>
            + approx::UlpsEq<Epsilon = T>
            + approx::RelativeEq<Epsilon = T>
            + approx::AbsDiffEq<Epsilon = T>,
    {
        Plane::get_plane_relaxed(aabb, T::default_epsilon(), T::default_max_ulps())
    }

    /// Try to figure out what axes defines the plane.
    /// If the AABB delta of one axis (a) is virtually nothing compared to
    /// the widest axis (b) while the third axis (c) is comparable to (b)
    /// by some fraction, we assume that that (a) isn't part of the plane.
    ///
    /// It's not possible to compare to zero exactly because blender
    /// leaves some decimal in coordinates that's suppose to be zero.
    pub fn get_plane_relaxed<T>(aabb: &Aabb3<T>, epsilon: T, max_ulps: u32) -> Option<Plane>
    where
        T: num_traits::Float
            + std::fmt::Debug
            + approx::AbsDiffEq
            + approx::UlpsEq
            + Sync
            + approx::AbsDiffEq<Epsilon = T>
            + approx::UlpsEq<Epsilon = T>
            + approx::RelativeEq<Epsilon = T>
            + approx::AbsDiffEq<Epsilon = T>,
    {
        if let Some(low_bound) = aabb.get_low() {
            if let Some(high_bound) = aabb.get_high() {
                let dx = high_bound[0] - low_bound[0];
                let dy = high_bound[1] - low_bound[1];
                let dz = high_bound[2] - low_bound[2];
                let max_delta = T::max(T::max(dx, dy), dz);

                let dx = T::zero().ulps_eq(&(dx / max_delta), epsilon, max_ulps);
                let dy = T::zero().ulps_eq(&(dy / max_delta), epsilon, max_ulps);
                let dz = T::zero().ulps_eq(&(dz / max_delta), epsilon, max_ulps);

                if dx && !dy && !dz {
                    return Some(Plane::ZY);
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
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    #[inline(always)]
    pub fn point_to_3d<T>(&self, point: &[T; 2]) -> [T; 3]
    where
        T: num_traits::Float
            + std::fmt::Debug
            + approx::AbsDiffEq
            + approx::UlpsEq
            + Sync
            + approx::AbsDiffEq<Epsilon = T>
            + approx::UlpsEq<Epsilon = T>
            + approx::RelativeEq<Epsilon = T>
            + approx::AbsDiffEq<Epsilon = T>,
    {
        match self {
            Plane::XY => [point[0], point[1], T::zero()],
            Plane::XZ => [point[0], T::zero(), point[1]],
            Plane::ZY => [T::zero(), point[1], point[0]],
        }
    }

    /// Copy this Point2 into a Point3, populating the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    #[allow(dead_code)]
    #[inline(always)]
    fn point_to_2d<T>(&self, point: &[T; 3]) -> [T; 2]
    where
        T: num_traits::Float
            + std::fmt::Debug
            + approx::AbsDiffEq
            + approx::UlpsEq
            + Sync
            + approx::AbsDiffEq<Epsilon = T>
            + approx::UlpsEq<Epsilon = T>
            + approx::RelativeEq<Epsilon = T>
            + approx::AbsDiffEq<Epsilon = T>,
    {
        match self {
            Plane::XY => [point[0], point[1]],
            Plane::XZ => [point[0], point[2]],
            Plane::ZY => [point[2], point[1]],
        }
    }
}

/// A 3d line
#[derive(PartialEq, Eq, Copy, Clone, Hash, fmt::Debug)]
pub struct Line3<T>
where
    T: num_traits::Float
        + std::fmt::Debug
        + approx::AbsDiffEq
        + approx::UlpsEq
        + Sync
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>,
{
    pub start: [T; 3],
    pub end: [T; 3],
}

impl<T> Line3<T>
where
    T: num_traits::Float
        + std::fmt::Debug
        + approx::AbsDiffEq
        + approx::UlpsEq
        + Sync
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>,
{
    pub fn new(start: [T; 3], end: [T; 3]) -> Self {
        Self { start, end }
    }

    /// returns (area of a triangle)²*4
    pub fn triangle_area_squared_times_4(p1: &[T; 3], p2: &[T; 3], p3: &[T; 3]) -> T {
        let v1_x = p1[0] - p2[0];
        let v1_y = p1[1] - p2[1];
        let v1_z = p1[2] - p2[2];

        let v2_x = p3[0] - p2[0];
        let v2_y = p3[1] - p2[1];
        let v2_z = p3[2] - p2[2];

        let x = v1_y * v2_z - v2_y * v1_z;
        let y = v1_x * v2_z - v2_x * v1_z;
        let z = v1_x * v2_y - v2_x * v1_y;
        x * x + y * y + z * z
    }

    /// Return a new line containing the matrix transformed points
    #[cfg(feature = "vecmath")]
    pub fn transform(&self, matrix4x4: &vecmath::Matrix4<T>) -> Self {
        Self {
            start: crate::vecmath_3d::col_mat4_transform_pos3(matrix4x4, self.start),
            end: crate::vecmath_3d::col_mat4_transform_pos3(matrix4x4, self.end),
        }
    }
}

#[allow(clippy::from_over_into)]
impl<T> Into<[T; 6]> for Line3<T>
where
    T: num_traits::Float
        + std::fmt::Debug
        + approx::AbsDiffEq
        + approx::UlpsEq
        + Sync
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>,
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
    T: num_traits::Float
        + std::fmt::Debug
        + approx::AbsDiffEq
        + approx::UlpsEq
        + Sync
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>,
    IT: Copy + Into<[T; 3]>,
{
    fn from(pos: [IT; 2]) -> Line3<T> {
        Line3::<T>::new(pos[0].into(), pos[1].into())
    }
}

impl<T> From<[T; 6]> for Line3<T>
where
    T: num_traits::Float
        + std::fmt::Debug
        + approx::AbsDiffEq
        + approx::UlpsEq
        + Sync
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>,
{
    fn from(l: [T; 6]) -> Line3<T> {
        Line3::<T>::new([l[0], l[1], l[2]], [l[3], l[4], l[5]])
    }
}

/// A 3d line string, aka polyline.
/// If the 'connected' field is set the 'as_lines()' method will connect start point with the
/// end-point.
/// Todo: The builder structure of this struct needs to be revisited
#[derive(PartialEq, Eq, Clone, Hash, fmt::Debug)]
pub struct LineString3<T>
where
    T: num_traits::Float
        + std::fmt::Debug
        + approx::AbsDiffEq
        + approx::UlpsEq
        + Sync
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>,
{
    pub(crate) points: Vec<[T; 3]>,

    /// if connected is set the as_lines() method will add an extra line connecting
    /// the first and last point
    pub connected: bool,
}

/// A set of linestrings + an aabb
/// Intended to contain related 3d shapes. E.g. outlines of letters with holes
#[derive(PartialEq, Eq, Clone, Hash)]
pub struct LineStringSet3<T>
where
    T: num_traits::Float
        + std::fmt::Debug
        + approx::AbsDiffEq
        + approx::UlpsEq
        + Sync
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>,
{
    pub set: Vec<LineString3<T>>,
    pub aabb: Aabb3<T>,
}

/// A simple 3d AABB
/// If min_max is none the data has not been assigned yet.
#[derive(PartialEq, Eq, Copy, Clone, Hash, fmt::Debug)]
pub struct Aabb3<T>
where
    T: num_traits::Float
        + std::fmt::Debug
        + approx::AbsDiffEq
        + approx::UlpsEq
        + Sync
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>,
{
    min_max: Option<([T; 3], [T; 3])>,
}

struct PriorityDistance<T> {
    key: T,
    index: usize,
}

impl<T> PartialOrd for PriorityDistance<T>
where
    T: PartialOrd
        + PartialEq
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>,
{
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl<T> Ord for PriorityDistance<T>
where
    T: PartialOrd
        + PartialEq
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>,
{
    #[inline(always)]
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.key.partial_cmp(&other.key).unwrap().reverse()
    }
}

impl<T> PartialEq for PriorityDistance<T>
where
    T: PartialOrd
        + PartialEq
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>,
{
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        ulps_eq!(&self.key, &other.key)
    }
}

impl<T> Eq for PriorityDistance<T> where
    T: PartialOrd
        + PartialEq
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>
{
}

impl<T> LineString3<T>
where
    T: num_traits::Float
        + std::fmt::Debug
        + approx::AbsDiffEq
        + approx::UlpsEq
        + Sync
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>,
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

    pub fn with_points(mut self, points: Vec<[T; 3]>) -> Self {
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
    pub fn copy_to_2d(&self, plane: Plane) -> vector_2d::LineString2<T> {
        let mut rv: vector_2d::LineString2<T> = match plane {
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

    /// Returns the line string as a Vec of lines
    /// Will be deprecated at some time, use self.as_lines_iter().collect() instead
    #[inline(always)]
    pub fn as_lines(&self) -> Vec<Line3<T>> {
        self.as_lines_iter().collect()
    }

    /// Returns the line string as a iterator of lines
    #[must_use = "iterator adaptors are lazy and do nothing unless consumed"]
    pub fn as_lines_iter(&self) -> Box<dyn Iterator<Item = Line3<T>> + '_> {
        if self.connected {
            Box::new(
                self.points
                    .iter()
                    .chain(self.points.last())
                    .tuple_windows::<(_, _)>()
                    .map(|(a, b)| Line3 { start: *a, end: *b }),
            )
        } else {
            Box::new(
                self.points
                    .iter()
                    .tuple_windows::<(_, _)>()
                    .map(|(a, b)| Line3 { start: *a, end: *b }),
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

    pub fn push(&mut self, point: [T; 3]) {
        self.points.push(point);
    }

    /// Return a new LineString3 containing the matrix transformed points
    #[cfg(feature = "vecmath")]
    pub fn transform(&self, matrix4x4: &vecmath::Matrix4<T>) -> Self {
        Self {
            points: self
                .points
                .iter()
                .map(|x| crate::vecmath_3d::col_mat4_transform_pos3(matrix4x4, *x))
                .collect(),
            connected: self.connected,
        }
    }

    /// Moves all the elements of `other` into `Self`, leaving `other` empty.
    /// TODO: currently ignores if `other` is connected or not.
    /// # Panics
    /// Panics if the number of elements in the points vector overflows a `usize`.
    pub fn append(&mut self, mut other: Self) {
        self.points.append(&mut other.points);
    }

    /// Simplify using Ramer–Douglas–Peucker algorithm adapted for 3d
    pub fn simplify(&self, distance_predicate: T) -> Self {
        //println!("input dist:{:?} slice{:?}", distance_predicate, self.points);

        if self.points.len() <= 2 {
            return self.clone();
        }
        if self.connected {
            let mut points = self.points.clone();
            // add the start-point to the end
            points.push(*points.first().unwrap());

            let mut rv: Vec<[T; 3]> = Vec::with_capacity(points.len());
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
            let mut rv: Vec<[T; 3]> = Vec::with_capacity(self.points.len());
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
    fn _simplify(distance_predicate_sq: T, slice: &[[T; 3]]) -> Vec<[T; 3]> {
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

    /// Simplify using Visvalingam–Whyatt algorithm adapted for 3d.
    /// This algorithm will delete 'points_to_delete' of points from the polyline with the smallest
    /// area defined by one point and it's neighbours.
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
                let area = Line3::triangle_area_squared_times_4(i.1, j.1, k.1);
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
            let area = Line3::triangle_area_squared_times_4(
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
                            let area = Line3::triangle_area_squared_times_4(
                                &self.points[prev],
                                &self.points[next % self_points_len],
                                &self.points[next_next % self_points_len],
                            );
                            let _ = search_tree.push(PriorityDistance {
                                key: area,
                                index: next,
                            });
                            let _ = link_tree.insert(next, (prev, next_next, area));

                            let area = Line3::triangle_area_squared_times_4(
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
                        let area = Line3::triangle_area_squared_times_4(
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
                        let area = Line3::triangle_area_squared_times_4(
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
}

impl<T, IC: Into<[T; 3]>> std::iter::FromIterator<IC> for LineString3<T>
where
    T: num_traits::Float
        + std::fmt::Debug
        + approx::AbsDiffEq
        + approx::UlpsEq
        + Sync
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>,
{
    fn from_iter<I: IntoIterator<Item = IC>>(iter: I) -> Self {
        LineString3 {
            points: iter.into_iter().map(|c| c.into()).collect(),
            connected: false,
        }
    }
}

impl<T> LineStringSet3<T>
where
    T: num_traits::Float
        + std::fmt::Debug
        + approx::AbsDiffEq
        + approx::UlpsEq
        + Sync
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>,
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

    #[cfg(feature = "vecmath")]
    pub fn transform(&self, mat: &vecmath::Matrix4<T>) -> Self {
        Self {
            set: self.set.iter().map(|x| x.transform(mat)).collect(),
            aabb: self.aabb.transform(mat),
        }
    }

    /// Copy this linestringset3 into a linestringset2, populating the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    pub fn copy_to_2d(&self, plane: Plane) -> vector_2d::LineStringSet2<T> {
        let mut rv = vector_2d::LineStringSet2::with_capacity(self.set.len());
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

impl<T, IT> From<[IT; 2]> for Aabb3<T>
where
    T: num_traits::Float
        + std::fmt::Debug
        + approx::AbsDiffEq
        + approx::UlpsEq
        + Sync
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>,
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
    T: num_traits::Float
        + std::fmt::Debug
        + approx::AbsDiffEq
        + approx::UlpsEq
        + Sync
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>,
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
    T: num_traits::Float
        + std::fmt::Debug
        + approx::AbsDiffEq
        + approx::UlpsEq
        + Sync
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>,
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

    #[cfg(feature = "vecmath")]
    pub fn transform(&self, matrix4x4: &vecmath::Matrix4<T>) -> Self {
        if let Some(min_max) = self.min_max {
            Self {
                min_max: Some((
                    crate::vecmath_3d::col_mat4_transform_pos3(matrix4x4, min_max.0),
                    crate::vecmath_3d::col_mat4_transform_pos3(matrix4x4, min_max.1),
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

#[inline(always)]
pub fn point_ulps_eq<T>(a: &[T; 3], b: &[T; 3]) -> bool
where
    T: num_traits::Float
        + std::fmt::Debug
        + approx::AbsDiffEq
        + approx::UlpsEq
        + Sync
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>,
{
    ulps_eq!(&a[0], &b[0]) && ulps_eq!(&a[1], &b[1]) && ulps_eq!(&a[2], &b[2])
}

#[inline(always)]
/// subtracts point b from point a resulting in a vector
fn sub<T>(a: &[T; 3], b: &[T; 3]) -> [T; 3]
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq + Sync,
{
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

#[allow(clippy::many_single_char_names)]
#[inline(always)]
fn cross_abs_squared<T>(a: &[T; 3], b: &[T; 3]) -> T
where
    T: num_traits::Float
        + std::fmt::Debug
        + approx::AbsDiffEq
        + approx::UlpsEq
        + Sync
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>,
{
    let x = a[1] * b[2] - a[2] * b[1];
    let y = a[2] * b[0] - a[0] * b[2];
    let z = a[0] * b[1] - a[1] * b[0];
    x * x + y * y + z * z
}

#[inline(always)]
/// The distance between the line a->b to the point p is the same as
/// distance = |(a-p)×(a-b)|/|a-b|
/// <https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Another_vector_formulation>
/// Make sure to *not* call this function with a-b==0
/// This function returns the distance²
pub fn distance_to_line_squared<T>(a: &[T; 3], b: &[T; 3], p: &[T; 3]) -> T
where
    T: num_traits::Float
        + std::fmt::Debug
        + approx::AbsDiffEq
        + approx::UlpsEq
        + Sync
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>,
{
    let a_sub_b = sub(a, b);
    let a_sub_p = sub(a, p);
    let a_sub_p_cross_a_sub_b_squared = cross_abs_squared(&a_sub_p, &a_sub_b);
    a_sub_p_cross_a_sub_b_squared
        / (a_sub_b[0] * a_sub_b[0] + a_sub_b[1] * a_sub_b[1] + a_sub_b[2] * a_sub_b[2])
}

/// Same as distance_to_line_squared<T> but it can be called when a-b might be 0.
/// It's a little slower because it does the a==b test
#[inline(always)]
pub fn distance_to_line_squared_safe<T>(a: &[T; 3], b: &[T; 3], p: &[T; 3]) -> T
where
    T: num_traits::Float
        + std::fmt::Debug
        + approx::AbsDiffEq
        + approx::UlpsEq
        + Sync
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>,
{
    if point_ulps_eq(a, b) {
        // give the point-to-point answer if the segment is a point
        distance_to_point_squared(a, p)
    } else {
        let a_sub_b = sub(a, b);
        let a_sub_p = sub(a, p);
        let a_sub_p_cross_a_sub_b_squared = cross_abs_squared(&a_sub_p, &a_sub_b);
        a_sub_p_cross_a_sub_b_squared
            / (a_sub_b[0] * a_sub_b[0] + a_sub_b[1] * a_sub_b[1] + a_sub_b[2] * a_sub_b[2])
    }
}

#[inline(always)]
/// The distance² between the two points
pub fn distance_to_point_squared<T>(a: &[T; 3], b: &[T; 3]) -> T
where
    T: num_traits::Float
        + std::fmt::Debug
        + approx::AbsDiffEq
        + approx::UlpsEq
        + Sync
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>,
{
    let v = sub(a, b);
    v[0] * v[0] + v[1] * v[1] + v[2] * v[2]
}

/// Rudimentary save line strings to .obj file function
/// It is extremely inefficient, but it works.
/// It saves with 6 decimals because that's what Blender uses.
pub fn save_to_obj_file<T>(
    filename: &str,
    object_name: &str,
    lines: Vec<Vec<Line3<T>>>,
) -> Result<(), LinestringError>
where
    T: num_traits::Float
        + std::fmt::Debug
        + approx::AbsDiffEq
        + approx::UlpsEq
        + Sync
        + approx::AbsDiffEq<Epsilon = T>
        + approx::UlpsEq<Epsilon = T>
        + approx::RelativeEq<Epsilon = T>
        + fmt::Display,
{
    let mut point_set = collections::HashMap::<String, usize>::new();
    // try to de-duplicate points
    for i in lines.iter() {
        for j in i.iter() {
            let a_str = format!("v {:.6} {:.6} {:.6}", j.start[0], j.start[1], j.start[2]);
            let len = point_set.len();
            let _ = point_set.entry(a_str).or_insert(len);
            let a_str = format!("v {:.6} {:.6} {:.6}", j.end[0], j.end[1], j.end[2]);
            let len = point_set.len();
            let _ = point_set.entry(a_str).or_insert(len);
        }
    }
    let path = path::Path::new(filename);
    let mut file = io::BufWriter::new(fs::File::create(&path)?);
    writeln!(file, "o {}", object_name)?;
    for (k, v) in point_set.iter().sorted_unstable_by(|a, b| a.1.cmp(b.1)) {
        writeln!(file, "{} #{}", k, v + 1)?;
    }
    for i in lines.iter() {
        for j in i.iter() {
            let str0 = format!("v {:.6} {:.6} {:.6}", j.start[0], j.start[1], j.start[2]);
            if let Some(p0) = point_set.get(&str0) {
                let str1 = format!("v {:.6} {:.6} {:.6}", j.end[0], j.end[1], j.end[2]);
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
