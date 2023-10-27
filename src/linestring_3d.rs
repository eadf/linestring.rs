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

use crate::{
    linestring_2d::{LineString2, LineStringSet2},
    LinestringError,
};
use itertools::Itertools;
use std::{collections, fmt, fs, hash::Hash, io, io::Write, path};
use vector_traits::{
    approx::{ulps_eq, AbsDiffEq, UlpsEq},
    num_traits::real::Real,
    GenericScalar, GenericVector3, HasXY,
};

/// Placeholder for different 3d shapes
pub enum Shape3d<T: GenericVector3> {
    Line(Line3<T>),
    Linestring(LineString3<T>),
    ParabolicArc(crate::linestring_2d::VoronoiParabolicArc<T::Vector2>),
}

/// Axis aligned planes, used to describe how imported 'flat' data is arranged in space
#[allow(clippy::upper_case_acronyms)]
#[derive(fmt::Debug, Copy, Clone)]
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
    //#[allow(dead_code)]
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
    pub fn triangle_area_squared_times_4(p1: &T, p2: &T, p3: &T) -> T::Scalar {
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

#[allow(clippy::from_over_into)]
impl<T: GenericVector3> Into<[T::Scalar; 6]> for Line3<T> {
    fn into(self) -> [T::Scalar; 6] {
        [
            self.start.x(),
            self.start.y(),
            self.start.z(),
            self.end.x(),
            self.end.y(),
            self.end.z(),
        ]
    }
}

impl<T: GenericVector3, IT: Copy + Into<T>> From<[IT; 2]> for Line3<T> {
    fn from(pos: [IT; 2]) -> Line3<T> {
        Line3::<T>::new(pos[0].into(), pos[1].into())
    }
}

impl<T: GenericVector3> From<[T::Scalar; 6]> for Line3<T> {
    fn from(l: [T::Scalar; 6]) -> Line3<T> {
        Line3::<T>::new(T::new_3d(l[0], l[1], l[2]), T::new_3d(l[3], l[4], l[5]))
    }
}

/// A 3d line string, aka polyline.
/// If the 'connected' field is set the 'as_lines()' method will connect start point with the
/// end-point.
/// Todo: The builder structure of this struct needs to be revisited
/// Todo: Remove the `connected` flag, just like LineString2
#[derive(PartialEq, Eq, Clone, Hash, fmt::Debug)]
pub struct LineString3<T: GenericVector3> {
    pub(crate) points: Vec<T>,

    /// if connected is set the as_lines() method will add an extra line connecting
    /// the first and last point
    pub connected: bool,
}

/// A set of linestrings + an aabb
/// Intended to contain related 3d shapes. E.g. outlines of letters with holes
#[derive(PartialEq, Eq, Clone, Hash)]
pub struct LineStringSet3<T: GenericVector3> {
    pub set: Vec<LineString3<T>>,
    pub aabb: Aabb3<T>,
}

/// A simple 3d AABB
/// If min_max is none no data has not been assigned yet.
#[derive(PartialEq, Eq, Copy, Clone, Hash, fmt::Debug)]
pub struct Aabb3<T: GenericVector3> {
    min_max: Option<(T, T)>,
}

impl<T: GenericVector3> Aabb3<T> {
    /// Apply an operation over each coordinate.
    pub fn apply<F: Fn(T) -> T>(mut self, f: &F) -> Self {
        if let Some(ref mut min_max) = self.min_max {
            self.min_max = Some((f(min_max.0), f(min_max.1)))
        }
        self
    }
}

struct PriorityDistance<T: GenericVector3> {
    key: T::Scalar,
    index: usize,
}

impl<T: GenericVector3> PartialOrd for PriorityDistance<T> {
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl<T: GenericVector3> Ord for PriorityDistance<T> {
    #[inline(always)]
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.key.partial_cmp(&other.key).unwrap().reverse()
    }
}

impl<T: GenericVector3> PartialEq for PriorityDistance<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        ulps_eq!(self.key, other.key)
    }
}

impl<T: GenericVector3> Eq for PriorityDistance<T> {}

impl<T: GenericVector3> LineString3<T> {
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::<T>::with_capacity(capacity),
            connected: false,
        }
    }

    pub fn with_points(mut self, points: Vec<T>) -> Self {
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
        I: Iterator<Item = &'a T>,
    {
        Self {
            points: iter.into_iter().copied().collect(),
            connected: false,
        }
    }

    /// Copy this linestring3 into a linestring2, keeping the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    pub fn copy_to_2d(&self, plane: Plane) -> LineString2<T::Vector2> {
        match plane {
            Plane::XY => self
                .points
                .iter()
                .map(|p3d| T::Vector2::new_2d(p3d.x(), p3d.y()))
                .collect(),
            Plane::XZ => self
                .points
                .iter()
                .map(|p3d| T::Vector2::new_2d(p3d.x(), p3d.z()))
                .collect(),
            Plane::YZ => self
                .points
                .iter()
                .map(|p3d| T::Vector2::new_2d(p3d.z(), p3d.y()))
                .collect(),
        }
    }

    pub fn points(&self) -> &Vec<T> {
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

    pub fn push(&mut self, point: T) {
        self.points.push(point);
    }

    /// Moves all the elements of `other` into `Self`, leaving `other` empty.
    /// TODO: currently ignores if `other` is connected or not.
    /// # Panics
    /// Panics if the number of elements in the points vector overflows a `usize`.
    pub fn append(&mut self, mut other: Self) {
        self.points.append(&mut other.points);
    }

    /// Simplify using Ramer–Douglas–Peucker algorithm adapted for 3d
    pub fn simplify_rdp(&self, distance_predicate: T::Scalar) -> Self {
        //println!("input dist:{:?} slice{:?}", distance_predicate, self.points);

        if self.points.len() <= 2 {
            return self.clone();
        }
        if self.connected {
            let mut points = self.points.clone();
            // add the start-point to the end
            points.push(*points.first().unwrap());

            let mut rv: Vec<T> = Vec::with_capacity(points.len());
            // _simplify() always omits the the first point of the result, so we have to add that
            rv.push(*points.first().unwrap());
            rv.append(&mut Self::_simplify_rdp(
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
            let mut rv: Vec<T> = Vec::with_capacity(self.points.len());
            // _simplify() always omits the the first point of the result, so we have to add that
            rv.push(*self.points.first().unwrap());
            rv.append(&mut Self::_simplify_rdp(
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
    fn _simplify_rdp(distance_predicate_sq: T::Scalar, slice: &[T]) -> Vec<T> {
        //println!("input dist:{:?} slice{:?}", distance_predicate_sq, slice);
        if slice.len() <= 2 {
            return slice[1..].to_vec();
        }
        // unwrap is safe since we tested len()>2
        let start_point = *slice.first().unwrap();
        let end_point = *slice.last().unwrap();

        let mut max_dist_sq = (-T::Scalar::ONE, 0_usize);
        let mut found_something = false;
        // find the point with largest distance to start_point<->endpoint line
        for (i, point) in slice.iter().enumerate().take(slice.len() - 1).skip(1) {
            let sq_d = distance_to_line_squared_safe(start_point, end_point, *point);

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
            return vec![end_point];
        }

        let mut rv = Self::_simplify_rdp(distance_predicate_sq, &slice[..max_dist_sq.1 + 1]);
        rv.append(&mut Self::_simplify_rdp(
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
        let mut link_tree = ahash::AHashMap::<usize, (usize, usize, T::Scalar)>::default();
        {
            let mut iter_i = self.points.iter().enumerate();
            let mut iter_j = self.points.iter().enumerate().skip(1);
            // the k iterator will terminate before i & j, so the iter_i & iter_j unwrap()s are safe
            for k in self.points.iter().enumerate().skip(2) {
                let i = iter_i.next().unwrap();
                let j = iter_j.next().unwrap();
                // define the area between point i, j & k as search criteria
                let area = Line3::triangle_area_squared_times_4(i.1, j.1, k.1);
                search_tree.push(PriorityDistance {
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
            search_tree.push(PriorityDistance {
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
                            search_tree.push(PriorityDistance {
                                key: area,
                                index: next,
                            });
                            let _ = link_tree.insert(next, (prev, next_next, area));

                            let area = Line3::triangle_area_squared_times_4(
                                &self.points[prev_prev],
                                &self.points[prev],
                                &self.points[next % self_points_len],
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
                        let area = Line3::triangle_area_squared_times_4(
                            &self.points[prev_prev],
                            &self.points[prev],
                            &self.points[next % self_points_len],
                        );
                        search_tree.push(PriorityDistance {
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
    pub fn apply<F: Fn(T) -> T>(mut self, f: &F) -> Self {
        self.points.iter_mut().for_each(|v| *v = f(*v));
        self
    }
}

impl<T: GenericVector3, IC: Into<T>> FromIterator<IC> for LineString3<T> {
    fn from_iter<I: IntoIterator<Item = IC>>(iter: I) -> Self {
        LineString3 {
            points: iter.into_iter().map(|c| c.into()).collect(),
            connected: false,
        }
    }
}

impl<T: GenericVector3> LineStringSet3<T> {
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            set: Vec::<LineString3<T>>::with_capacity(capacity),
            aabb: Aabb3::<T>::default(),
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
                self.aabb.update_point(*ls);
            }
        }
    }

    pub fn get_aabb(&self) -> Aabb3<T> {
        self.aabb
    }

    pub fn apply<F: Fn(T) -> T>(self, f: &F) -> Self {
        Self {
            set: self.set.into_iter().map(|x| x.apply(f)).collect(),
            aabb: self.aabb.apply(f),
        }
    }

    /// Copy this linestringset3 into a linestringset2, populating the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    pub fn copy_to_2d(&self, plane: Plane) -> LineStringSet2<T::Vector2> {
        let mut rv = LineStringSet2::with_capacity(self.set.len());
        for ls in self.set.iter() {
            rv.push(ls.copy_to_2d(plane));
        }
        rv
    }

    /// drains the 'other' container of all shapes and put them into 'self'
    pub fn take_from(&mut self, other: &mut Self) {
        self.aabb.update_aabb(other.aabb);
        self.set.append(&mut other.set);
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
            self.update_point(min);
            self.update_point(max);
        }
    }

    pub fn update_point(&mut self, point: T) {
        if self.min_max.is_none() {
            self.min_max = Some((point, point));
            return;
        }
        let (mut aabb_min, mut aabb_max) = self.min_max.take().unwrap();

        if point.x() < aabb_min.x() {
            aabb_min.set_x( point.x());
        }
        if point.y() < aabb_min.y() {
            aabb_min.set_y( point.y());
        }
        if point.z() < aabb_min.z() {
            aabb_min.set_z( point.z());
        }
        if point.x() > aabb_max.x() {
            aabb_max.set_x( point.x());
        }
        if point.y() > aabb_max.y() {
            aabb_max.set_y( point.y());
        }
        if point.z() > aabb_max.z() {
            aabb_max.set_z( point.z());
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

/// Same as distance_to_line_squared<T> but it can be called when a-b might be 0.
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
/// It is extremely inefficient, but it works.
pub fn save_to_obj_file<T: GenericVector3>(
    filename: &str,
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
    let path = path::Path::new(filename);
    let mut file = io::BufWriter::new(fs::File::create(path)?);
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
