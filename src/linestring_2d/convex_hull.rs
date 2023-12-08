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
#[cfg(test)]
mod tests;

use crate::{linestring_2d::LineString2, LinestringError};
use itertools::Itertools;
use rayon::prelude::*;
use std::cmp::Ordering;
use vector_traits::{approx::ulps_eq, num_traits::real::Real, GenericScalar, GenericVector2};

#[derive(PartialEq, Clone, Copy)]
pub enum Orientation {
    Left,
    Right,
    Collinear,
}

impl Orientation {
    #[inline(always)]
    pub fn is_left_or_collinear(self) -> bool {
        self == Orientation::Left || self == Orientation::Collinear
    }

    #[inline(always)]
    pub fn is_left(self) -> bool {
        self == Orientation::Left
    }
}

/// finds the point with lowest x, if x is equal -> find the point with lowest y value
fn find_lowest_x<T: GenericVector2>(vertices: &[T]) -> Result<(usize, T), LinestringError> {
    if vertices.is_empty() {
        return Err(LinestringError::InvalidData(
            "No points to compare".to_string(),
        ));
    }

    let mut min_idx = 0;
    let mut min_val = vertices[0];

    for (idx, point) in vertices.iter().enumerate().skip(1) {
        match point.x().partial_cmp(&min_val.x()) {
            Some(Ordering::Less) => {
                min_idx = idx;
                min_val = *point;
            }
            Some(Ordering::Equal) => {
                if point.y() < min_val.y() {
                    min_idx = idx;
                    min_val = *point;
                }
            }
            Some(Ordering::Greater) => (),
            _ => {
                return Err(LinestringError::InvalidData(
                    format!(
                        "Comparison failed for x-coordinate ({},{}) vs ({},{})",
                        point.x(),
                        point.y(),
                        min_val.x(),
                        min_val.y()
                    )
                    .to_string(),
                ));
            }
        }
    }

    Ok((min_idx, min_val))
}

/// finds the point with lowest x, if x is equal -> find the point with lowest y value
/// returns the index out of `indices` that, in turn, indicate the lowest value vertex.
/// i.e. `vertices[indices[lowest_index]]` -> lowest vertex of `vertices`
fn indexed_find_lowest_x<T: GenericVector2>(
    vertices: &[T],
    indices: &[usize],
) -> Result<(usize, T), LinestringError> {
    if vertices.is_empty() || indices.is_empty() {
        return Err(LinestringError::InvalidData(
            "No points to compare".to_string(),
        ));
    }

    let mut min_idx_idx = 0;
    let mut min_val = vertices[indices[min_idx_idx]];

    for (index_index, index) in indices.iter().enumerate().skip(1) {
        let point = vertices[*index];
        match point.x().partial_cmp(&min_val.x()) {
            Some(Ordering::Less) => {
                min_idx_idx = index_index;
                min_val = point;
            }
            Some(Ordering::Equal) => {
                if point.y() < min_val.y() {
                    min_idx_idx = index_index;
                    min_val = point;
                }
            }
            Some(Ordering::Greater) => (),
            _ => {
                return Err(LinestringError::InvalidData(
                    format!(
                        "Comparison failed for x-coordinate ({},{}) vs ({},{})",
                        point.x(),
                        point.y(),
                        min_val.x(),
                        min_val.y()
                    )
                    .to_string(),
                ));
            }
        }
    }

    Ok((min_idx_idx, min_val))
}

/// Returns an indication if the point 'c' lies to the 'left' of the line a->b
///```
/// # use linestring::linestring_2d::convex_hull;
/// # use vector_traits::glam::Vec2;
/// let a = Vec2 { x: 0.0, y: 0.0 };
/// let b = Vec2 { x: 0.0, y: 10.0 };
/// let c = Vec2 { x: -10.0, y: 5.0 };
/// assert!(convex_hull::point_orientation(a, b, c).is_left());
/// assert!(!convex_hull::point_orientation(a, c, b).is_left());
/// assert!(convex_hull::point_orientation(c, a, b).is_left());
/// assert!(!convex_hull::point_orientation(c, b, a).is_left());
///```
#[inline(always)]
pub fn point_orientation<T: GenericVector2>(a: T, b: T, c: T) -> Orientation {
    let value = cross_2d(a, b, c);
    if value == T::Scalar::ZERO || ulps_eq!(value, T::Scalar::ZERO) {
        Orientation::Collinear
    } else if value > T::Scalar::ZERO {
        Orientation::Left
    } else {
        Orientation::Right
    }
}

/// Returns true if the point 'c' lies to the 'left' of the line a->b
///```
/// # use linestring::linestring_2d::convex_hull;
/// # use vector_traits::glam::Vec2;
/// let a = Vec2 { x: 0.0, y: 0.0 };
/// let b = Vec2 { x: 0.0, y: 10.0 };
/// let c = Vec2 { x: -10.0, y: 5.0 };
/// assert!(convex_hull::is_point_left(a, b, c));
/// assert!(!convex_hull::is_point_left(a, c, b));
/// assert!(convex_hull::is_point_left(c, a, b));
/// assert!(!convex_hull::is_point_left(c, b, a));
///```
#[inline(always)]
pub fn is_point_left<T: GenericVector2>(a: T, b: T, c: T) -> bool {
    cross_2d(a, b, c) > T::Scalar::ZERO
}

/// Returns true if the point 'c' lies to the 'left' of the line a->b
/// Returns true even if the point lies on the line a-b
///```
/// # use linestring::linestring_2d::convex_hull;
/// # use vector_traits::glam::Vec2;
/// let a = Vec2 { x: 0.0_f32, y: 0.0 };
/// let b = Vec2 { x: 0.0_f32, y: 10.0 };
/// assert!(convex_hull::is_point_left_allow_collinear(a, b, b));
/// assert!(convex_hull::is_point_left_allow_collinear(a, b, a));
///```
#[inline(always)]
pub fn is_point_left_allow_collinear<T: GenericVector2>(a: T, b: T, c: T) -> bool {
    let result = cross_2d(a, b, c);
    result >= T::Scalar::ZERO || ulps_eq!(result, T::Scalar::ZERO)
}

/// calculate the cross product of two 2d vectors defined by the points a,b,c as: a->b & a->c
/// return the z coordinate as a scalar.
/// The return value will be positive if the point c is 'left' of the vector a->b (ccr)
///```
/// # use linestring::linestring_2d::convex_hull;
/// # use vector_traits::glam::Vec2;
/// let a = Vec2{x:0.0, y:0.0};
/// let b = Vec2{x:0.0, y:10.0};
/// let c = Vec2{x:-10.0, y:5.0};
/// assert!(convex_hull::cross_2d(a, b, c) > 0.0);
/// assert!(convex_hull::cross_2d(a, c, b) < 0.0);
/// assert!(convex_hull::cross_2d(c, a, b) > 0.0);
/// assert!(convex_hull::cross_2d(c, b, a) < 0.0);
///```
#[inline(always)]
pub fn cross_2d<T: GenericVector2>(a: T, b: T, c: T) -> T::Scalar {
    (b - a).perp_dot(c - a)
}

/// finds the convex hull using Gift wrapping algorithm
/// <https://en.wikipedia.org/wiki/Gift_wrapping_algorithm>
/// This implementation had a problem with never reaching the end condition under certain situations
/// so i fixed it with a `visited` set.
///```
/// # use linestring::linestring_2d;
/// # use linestring::linestring_2d::convex_hull;
/// # use vector_traits::glam::Vec2;
/// # use rand::{Rng, SeedableRng, rngs::StdRng};
/// use linestring::prelude::LineString2;
/// let mut rng:StdRng = SeedableRng::from_seed([42; 32]);
/// let mut points = Vec::<Vec2>::new();
/// for _i in 0..50 {
///   let p: [f32; 2] = [rng.gen_range(0.0..4096.0), rng.gen_range(0.0..4096.0)];
///   points.push(p.into());
/// }
///
/// let convex_hull:Vec<Vec2> = convex_hull::gift_wrap(&points)?;
/// let center = Vec2{x:2000_f32, y:2000.0};
///
/// for p in convex_hull.iter() {
///   for l in convex_hull.window_iter() {
///     // all segments should have the center point at the 'left' side
///     assert!(convex_hull::point_orientation(l.start, l.end, center).is_left());
///     assert!(convex_hull::is_point_left(l.start, l.end, center));
///     // all points on the hull should be 'left' of every segment in the hull
///     assert!(convex_hull::point_orientation(l.start, l.end, *p).is_left_or_collinear());
///   }
/// }
/// # Ok::<(), linestring::LinestringError>(())
///```
pub fn gift_wrap<T: GenericVector2>(mut vertices: &[T]) -> Result<Vec<T>, LinestringError> {
    if vertices.is_empty() {
        return Ok(Vec::with_capacity(0));
    }
    if vertices[0] == vertices[vertices.len() - 1] {
        // disregard the duplicated loop point if it exists
        vertices = &vertices[1..];
    }
    if vertices.len() <= 2 {
        return Ok(vertices.to_vec());
    }
    if vertices.len() == 3 {
        return if is_point_left(vertices[0], vertices[1], vertices[2]) {
            Ok(vertices.to_vec())
        } else {
            Ok(vec![vertices[0], vertices[2], vertices[1]])
        };
    }

    let starting_point = find_lowest_x(vertices)?.0;

    let mut hull = Vec::with_capacity(vertices.len() / 2);
    // To track visited points
    let mut visited: vob::Vob<u32> = vob::Vob::new_with_storage_type(vertices.len());
    visited.resize(vertices.len(), false);

    let mut point_on_hull = starting_point;
    //println!("starting point {:?}:{}", input_points[starting_point], starting_point);
    loop {
        hull.push(vertices[point_on_hull]);
        if point_on_hull != starting_point {
            // don't mark the starting_point or we won't know where to stop
            //println!("hull marking {:?}:{} as visited", input_points[point_on_hull], point_on_hull);
            let _ = visited.set(point_on_hull, true);
        }
        let mut end_point = (point_on_hull + 1) % vertices.len();

        for j in 0..vertices.len() {
            if j == point_on_hull || j == end_point || visited[j] {
                continue;
            }

            let orient =
                point_orientation(vertices[point_on_hull], vertices[end_point], vertices[j]);
            if orient == Orientation::Right {
                // replace end_point with the candidate if candidate is to the right,
                end_point = j;
            } else if orient == Orientation::Collinear {
                //println!("the points {:?}:{},{:?}:{},{:?}:{} were collinear", input_points[point_on_hull],point_on_hull, input_points[end_point], end_point, input_points[j],j );
                let distance_sq_candidate = vertices[point_on_hull].distance_sq(vertices[j]);
                let distance_sq_end_point =
                    vertices[point_on_hull].distance_sq(vertices[end_point]);

                if distance_sq_candidate > distance_sq_end_point {
                    // if point_on_hull->end_point && point_on_hull->candidate are collinear and
                    // the distance to the collinear is greater we also update end_point
                    end_point = j;
                } else if j != starting_point {
                    // This is a collinear point that is closer to the current point, mark it as visited
                    //println!("coll marking {:?}:{} as visited", input_points[j], j);
                    let _ = visited.set(j, true);
                }
            }
        }

        point_on_hull = end_point;
        if end_point == starting_point {
            // complete the loop, `hull` now represents a closed loop of points.
            // Each edge can be iterated over by `hull.iter().window(2)`
            hull.push(vertices[end_point]);
            break;
        }
    }

    Ok(hull)
}

/// finds the convex hull using the [Gift wrapping algorithm](https://en.wikipedia.org/wiki/Gift_wrapping_algorithm)
/// This implementation had a problem with never reaching the end condition under certain situations
/// so i fixed it with a `visited` set.
/// # Arguments
///
/// * `vertices` - The input vertices.
/// * `indices` - The selected indices to the vertices. Not all vertices need to be mentioned here;
/// only those in the indices list are used by the function.
///
/// # Returns
///
/// A list of indices pointing back into the vertices list, representing the convex hull.
/// The list will be closed, i.e the first and last element will be the same.
/// # Errors
///
/// Returns an error if any errors occur during computation.E.g. if any vertex component is NaN or infinite
///
/// # Example
///
///```
/// # use linestring::linestring_2d;
/// # use linestring::linestring_2d::convex_hull;
/// # use vector_traits::glam::{Vec2,vec2};
/// # use rand::{Rng, SeedableRng, rngs::StdRng};
/// let mut rng:StdRng = SeedableRng::from_seed([42; 32]);
/// let mut points = Vec::<Vec2>::new();
/// for _i in 0..200 {
///   let p: [f32; 2] = [rng.gen_range(0.0..10.0), rng.gen_range(0.0..10.0)];
///   points.push(p.into());
/// }
///
/// let indices:Vec<usize> = (0..points.len()).collect();
/// let convex_hull = convex_hull::indexed_gift_wrap(&points, &indices)?;
/// let center = vec2(5_f32, 5.0);
///
/// for p in &convex_hull {
///   for edge in convex_hull.windows(2) {
///     // all segments should have the center point at the 'left' side
///     assert!(convex_hull::point_orientation(points[edge[0]], points[edge[1]], center).is_left());
///     assert!(convex_hull::is_point_left(points[edge[0]], points[edge[1]], center));
///     // all points on the hull should be 'left' of every segment in the hull
///     assert!(convex_hull::point_orientation(points[edge[0]], points[edge[1]], points[*p]).is_left_or_collinear());
///   }
/// }
/// # Ok::<(), linestring::LinestringError>(())
///```
pub fn indexed_gift_wrap<T: GenericVector2>(
    vertices: &[T],
    indices: &[usize],
) -> Result<Vec<usize>, LinestringError> {
    let mut hull = indexed_gift_wrap_no_loop(vertices, indices)?;
    // complete the loop, `hull` now represents a closed loop of points.
    // Each edge can be iterated over by `rv.points.iter().window(2)`
    if hull.len() > 1 {
        let first = hull.first().unwrap();
        hull.push(*first);
    }
    Ok(hull)
}

/// does the same as indexed_gift_wrap() but does not close the loop
#[inline]
fn indexed_gift_wrap_no_loop<T: GenericVector2>(
    vertices: &[T],
    mut indices: &[usize],
) -> Result<Vec<usize>, LinestringError> {
    if indices.len() <= 1 {
        return Ok(indices.to_vec());
    }
    if indices.first().unwrap() == indices.last().unwrap() {
        // disregard the duplicated loop point if it exists
        indices = &indices[0..indices.len() - 1];
    }
    if indices.len() <= 2 {
        return Ok(indices.to_vec());
    }
    if indices.len() == 3 {
        return if is_point_left(
            vertices[indices[0]],
            vertices[indices[1]],
            vertices[indices[2]],
        ) {
            // input_indices was already CCW
            Ok(indices.to_vec())
        } else {
            // Convert from CW to CCW
            Ok(vec![indices[0], indices[2], indices[1]])
        };
    }
    let starting_index = indexed_find_lowest_x(vertices, indices)?.0;

    let mut hull = Vec::with_capacity(indices.len() / 4);
    // To track visited points
    let mut visited: vob::Vob<u32> = vob::Vob::new_with_storage_type(indices.len());
    visited.resize(indices.len(), false);

    let mut point_on_hull = starting_index;
    //println!("starting point {:?}:{}", input_points[starting_point], starting_point);
    loop {
        hull.push(indices[point_on_hull]);
        if point_on_hull != starting_index {
            // don't mark the starting_point or we won't know where to stop
            let _ = visited.set(point_on_hull, true);
        }
        let mut end_point = (point_on_hull + 1) % indices.len();

        for j in 0..indices.len() {
            if j == point_on_hull || j == end_point || visited[j] {
                continue;
            }

            let orient = point_orientation(
                vertices[indices[point_on_hull]],
                vertices[indices[end_point]],
                vertices[indices[j]],
            );
            if orient == Orientation::Right {
                // replace end_point with the candidate if candidate is to the right,
                end_point = j;
            } else if orient == Orientation::Collinear {
                let distance_sq_candidate =
                    vertices[indices[point_on_hull]].distance_sq(vertices[indices[j]]);
                let distance_sq_end_point =
                    vertices[indices[point_on_hull]].distance_sq(vertices[indices[end_point]]);

                if distance_sq_candidate > distance_sq_end_point {
                    // if point_on_hull->end_point && point_on_hull->candidate are collinear and
                    // the distance to the collinear is greater we also update end_point
                    end_point = j;
                } else if j != starting_index {
                    // This is a collinear point that is closer to the current point, mark it as visited
                    let _ = visited.set(j, true);
                }
            }
        }

        point_on_hull = end_point;
        if end_point == starting_index {
            break;
        }
    }
    /*println!("indexed_gift_wrap:");
    println!("starting_index {:?}", input_indices[starting_index]);
    println!("input {:?}", input_vertices);
    println!("input indices {:?}", input_indices);
    println!("result {:?}", hull);*/

    Ok(hull)
}

/// finds the convex hull using Grahams scan
/// <https://en.wikipedia.org/wiki/Graham_scan>
/// This is an experimental implementation that should not be used.
///```
/// # use linestring::linestring_2d::{LineString2,convex_hull};
/// # use vector_traits::glam::Vec2;
/// # use rand::{Rng, SeedableRng, rngs::StdRng};
/// let mut rng:StdRng = SeedableRng::from_seed([42; 32]);
/// let mut points = Vec::<Vec2>::new();
/// for _i in 0..1023 {
///   let p: [f32; 2] = [rng.gen_range(0.0..4096.0), rng.gen_range(0.0..4096.0)];
///   points.push(p.into());
/// }
///
/// #[allow(deprecated)]
/// let convex_hull = convex_hull::graham_scan_wo_atan2(&points)?;
/// let center = Vec2{x:2000_f32, y:2000.0};
///
/// for p in convex_hull.iter() {
///   for l in convex_hull.window_iter() {
///     // all segments should have the center point at the 'left' side
///     assert!(convex_hull::is_point_left(l.start, l.end, center));
///     // all points on the hull should be 'left' of every segment in the hull
///     assert!(convex_hull::is_point_left_allow_collinear(l.start, l.end, *p));
///   }
/// }
/// # Ok::<(), linestring::LinestringError>(())
///```
#[deprecated(since = "0.9.0", note = "please use `graham_scan` instead")]
pub fn graham_scan_wo_atan2<T: GenericVector2>(
    mut input_points: &[T],
) -> Result<Vec<T>, LinestringError> {
    if input_points.is_empty() {
        return Ok(Vec::with_capacity(0));
    }
    if !input_points.is_empty() && (input_points[0] == input_points[input_points.len() - 1]) {
        // disregard the duplicated loop point if it exists
        input_points = &input_points[1..];
    }
    if input_points.len() <= 2 {
        return Ok(input_points.to_vec());
    }
    if input_points.len() == 3 {
        return if is_point_left(input_points[0], input_points[1], input_points[2]) {
            Ok(input_points.to_vec())
        } else {
            Ok(vec![input_points[0], input_points[2], input_points[1]])
        };
    }

    let mut input_points = input_points.to_vec();

    let starting_point = find_lowest_x(&input_points)?.1;

    let comparator = |b: &T, c: &T| {
        let ccr = cross_2d(starting_point, *b, *c);
        match T::Scalar::ZERO.partial_cmp(&ccr) {
            Some(Ordering::Greater) => Ordering::Greater, // CCR
            Some(Ordering::Less) => Ordering::Less,       // CR
            Some(Ordering::Equal) => {
                // Collinear, pick the furthest node
                let dist_sb = starting_point.distance_sq(*b);
                let dist_sc = starting_point.distance_sq(*c);
                if dist_sb > dist_sc {
                    Ordering::Greater
                } else {
                    Ordering::Less
                }
            }
            _ => panic!(),
        }
    };
    // sort the input points so that the edges to the 'right' of starting_point goes first
    input_points.sort_unstable_by(comparator);

    let mut rv = Vec::with_capacity(input_points.len());
    rv.push(starting_point);

    for p in input_points.iter() {
        while rv.len() > 1 {
            let last = rv.len() - 1;
            let cross = cross_2d(rv[last - 1], rv[last], *p);
            if cross < T::Scalar::ZERO || ulps_eq!(cross, T::Scalar::ZERO) {
                let _ = rv.pop();
            } else {
                break;
            }
        }
        if rv.last().unwrap() != p {
            rv.push(*p);
        }
    }

    if !rv.is_connected() {
        let first = rv.first().unwrap();
        rv.push(*first)
    };
    Ok(rv)
}

/// finds the convex hull using Grahams scan
/// <https://en.wikipedia.org/wiki/Graham_scan>
///```
/// # use linestring::linestring_2d::{LineString2,convex_hull};
/// # use vector_traits::glam::Vec2;
/// # use rand::{Rng, SeedableRng, rngs::StdRng};
/// let mut rng:StdRng = SeedableRng::from_seed([42; 32]);
/// let mut points = Vec::<Vec2>::new();
/// for _i in 0..1023 {
///   let p: [f32; 2] = [rng.gen_range(0.0..4096.0), rng.gen_range(0.0..4096.0)];
///   points.push(p.into());
/// }
///
/// let convex_hull = convex_hull::graham_scan(&points)?;
/// let center = Vec2{x:2000_f32, y:2000.0};
///
/// for p in convex_hull.iter() {
///   for l in convex_hull.window_iter() {
///     // all segments should have the center point at the 'left' side
///     assert!(convex_hull::is_point_left(l.start, l.end, center));
///     // all points on the hull should be 'left' of every segment in the hull
///     assert!(convex_hull::is_point_left_allow_collinear(l.start, l.end, *p));
///   }
/// }
/// # Ok::<(), linestring::LinestringError>(())
///```
pub fn graham_scan<T: GenericVector2>(mut input_points: &[T]) -> Result<Vec<T>, LinestringError> {
    if input_points.is_empty() {
        return Ok(Vec::with_capacity(0));
    }
    if !input_points.is_empty() && (input_points[0] == input_points[input_points.len() - 1]) {
        // disregard the duplicated loop point if it exists
        input_points = &input_points[1..];
    }
    if input_points.len() <= 2 {
        return Ok(input_points.to_vec());
    }
    if input_points.len() == 3 {
        return if is_point_left(input_points[0], input_points[1], input_points[2]) {
            Ok(input_points.to_vec())
        } else {
            Ok(vec![input_points[0], input_points[2], input_points[1]])
        };
    }

    let (_, starting_point) = find_lowest_x(input_points)?;
    //let ref_dir = T::new(T::Scalar::ONE, T::Scalar::ZERO);
    let mut input_points: Vec<(T, T::Scalar, T::Scalar)> = input_points
        .iter()
        .map(|&p| {
            let diff = p - starting_point;
            let angle = diff.y().atan2(diff.x());
            let distance = starting_point.distance_sq(p);
            (p, angle, distance)
        })
        .collect();

    //println!("unsorted:{:?}", input_points);
    input_points.sort_unstable_by(|a, b| {
        a.1.partial_cmp(&b.1)
            .unwrap_or(Ordering::Equal)
            .then_with(|| a.2.partial_cmp(&b.2).unwrap_or(Ordering::Equal))
    });

    //println!("starting_point:{:?}", starting_point);
    //println!("sorted:{:?}", input_points);
    let mut hull = Vec::<T>::with_capacity(input_points.len());
    hull.push(starting_point);

    for (p, _, _) in input_points.iter() {
        if *p == starting_point {
            continue;
        }
        while hull.len() > 1 {
            let last = hull.len() - 1;
            let cross = cross_2d(hull[last - 1], hull[last], *p);
            if T::Scalar::ZERO > cross || ulps_eq!(cross, T::Scalar::ZERO) {
                let _ = hull.pop();
            } else {
                break;
            }
        }
        //println!("pushing p:{:?}", p);
        hull.push(*p);
    }
    //println!("result {:?}", rv);

    // complete the loop, `hull` now represents a closed loop of points.
    // Each edge can be iterated over by `rv.points.iter().window(2)`
    if hull.len() > 1 {
        hull.push(*hull.first().unwrap());
    }
    Ok(hull)
}

/// Finds the convex hull using [Graham's Scan](https://en.wikipedia.org/wiki/Graham_scan)
///
/// # Arguments
///
/// * `vertices` - The input vertices.
/// * `indices` - The selected indices to the vertices. Not all vertices need to be mentioned here;
/// only those in the indices list are used by the function.
///
/// # Returns
///
/// A list of indices pointing back into the vertices list, representing the convex hull.
/// The list will be closed, i.e the first and last element will be the same.
/// # Errors
///
/// Returns an error if any errors occur during computation.E.g. if any vertex component is NaN or infinite
///
/// # Example
///
///```
/// # use linestring::linestring_2d::convex_hull;
/// # use vector_traits::glam::Vec2;
/// # use rand::{Rng, SeedableRng, rngs::StdRng};
/// let mut rng:StdRng = SeedableRng::from_seed([42; 32]);
/// let mut vertices = Vec::<Vec2>::new();
/// for _i in 0..1023 {
///   let p: [f32; 2] = [rng.gen_range(0.0..4000.0), rng.gen_range(0.0..4000.0)];
///   vertices.push(p.into());
/// }
///
/// let indices:Vec<usize> = (0..vertices.len()).collect();
/// let convex_hull = convex_hull::indexed_graham_scan(&vertices, &indices)?;
/// let center = Vec2{x:2000_f32, y:2000.0};
///
/// for i in convex_hull.iter() {
///   for edge in convex_hull.chunks_exact(2) {
///     // all segments should have the center point at the 'left' side
///     assert!(convex_hull::is_point_left(vertices[edge[0]], vertices[edge[1]], center));
///     // all points on the hull should be 'left' of every segment in the hull
///     assert!(convex_hull::is_point_left_allow_collinear(vertices[edge[0]], vertices[edge[1]], vertices[*i]));
///   }
/// }
/// # Ok::<(), linestring::LinestringError>(())
///```
pub fn indexed_graham_scan<T: GenericVector2>(
    vertices: &[T],
    indices: &[usize],
) -> Result<Vec<usize>, LinestringError> {
    let mut hull = indexed_graham_scan_no_loop(vertices, indices, None)?;
    // complete the loop, `hull` now represents a closed loop of points.
    // Each edge can be iterated over by `rv.points.iter().window(2)`
    if hull.len() > 1 {
        hull.push(*hull.first().unwrap());
    }
    Ok(hull)
}

#[inline]
/// does the same as indexed_graham_scan() but does not close the loop
pub fn indexed_graham_scan_no_loop<T: GenericVector2>(
    vertices: &[T],
    mut indices: &[usize],
    start_index: Option<usize>,
) -> Result<Vec<usize>, LinestringError> {
    //println!("indexed_graham_scan: input {:?}", indices);

    if indices.len() <= 1 {
        return Ok(indices.to_vec());
    }
    if indices.first().unwrap() == indices.last().unwrap() {
        // disregard the duplicated loop point if it exists
        indices = &indices[0..indices.len() - 1];
    }
    if indices.len() <= 2 {
        return Ok(indices.to_vec());
    }
    if indices.len() == 3 {
        return if is_point_left(
            vertices[indices[0]],
            vertices[indices[1]],
            vertices[indices[2]],
        ) {
            // input_indices was already CCW
            Ok(indices.to_vec())
        } else {
            // Convert from CW to CCW
            Ok(vec![indices[0], indices[2], indices[1]])
        };
    }
    let (start_index, start_point) = if let Some(start_index) = start_index {
        (start_index, vertices[start_index])
    } else {
        let (start_index_index, start_point) = indexed_find_lowest_x(vertices, indices)?;
        (indices[start_index_index], start_point)
    };
    // sort all indices with regard to the angle to starting_point
    let sorted_indices: Vec<usize> = indices
        .iter()
        .map(|i| {
            let p = vertices[*i];
            let diff = p - start_point;
            let angle = diff.y().atan2(diff.x());
            let distance = start_point.distance_sq(p);
            (*i, angle, distance)
        })
        .sorted_unstable_by(|a, b| {
            a.1.partial_cmp(&b.1)
                .unwrap_or(Ordering::Equal)
                .then_with(|| a.2.partial_cmp(&b.2).unwrap_or(Ordering::Equal))
        })
        .map(|a| a.0)
        .collect();

    let mut hull = Vec::<usize>::with_capacity(vertices.len());
    hull.push(start_index);

    for i in sorted_indices.iter() {
        if *i == start_index {
            continue;
        }
        let p = vertices[*i];
        while hull.len() > 1 {
            let last_in_hull = hull.len() - 1;
            let cross = cross_2d(
                vertices[hull[last_in_hull - 1]],
                vertices[hull[last_in_hull]],
                p,
            );
            if T::Scalar::ZERO > cross || ulps_eq!(cross, T::Scalar::ZERO) {
                let _ = hull.pop();
            } else {
                break;
            }
        }
        hull.push(*i);
    }

    Ok(hull)
}

/*
/// Combines two convex hulls together as two point clouds using `indexed_gift_wrap_no_loop()`
fn combine_indexed_convex_hull_lazy<T: GenericVector2>(
    vertices: &[T],
    indices_a: Result<Vec<usize>, LinestringError>,
    indices_b: Result<Vec<usize>, LinestringError>,
) -> Result<Vec<usize>, LinestringError> {
    let mut indices_a = indices_a?;
    let indices_b = indices_b?;

    if indices_a.is_empty() {
        return Ok(indices_b);
    }
    if indices_b.is_empty() {
        return Ok(indices_a);
    }

    if indices_a.first().unwrap() == indices_b.first().unwrap() {
        indices_a.extend(indices_b.iter().skip(1));
    } else {
        indices_a.extend(indices_b.iter());
    }
    indexed_graham_scan_no_loop(vertices, &indices_a, None)
}*/

/// a structure trying to keep track of from what list indices came from.
#[derive(Clone)]
struct VertexIndex {
    /// `index_index` was from `indices_a` if true, else `indices_b`
    from_a: bool,
    /// the 'real' index into `vertices`
    index: usize,
    /// the index into `indices_a` or `indices_a`
    index_index: usize,
}

/// Combines two convex hulls together using a modified gift wrapping algorithm.
/// The algorithm goes like this.
/// We keep track of from what list the start_point, point_on_hull and endpoint comes from.
/// The algorithm is like gift-wrapping in that it keeps `point_on_hull` and `end_point` and iterates
/// though a list of `j` candidates to try to find a better end-point.
/// First it picks `end_point` to be from `ìndices_a`, then it iterates over `ìndices_b` trying to
/// find a better `end_point`. If an end point is found in `ìndices_b` but another invalid one is
/// found the inner loop ends.
fn combine_indexed_convex_hull<T: GenericVector2>(
    vertices: &[T],
    indices_a: Result<Vec<usize>, LinestringError>,
    indices_b: Result<Vec<usize>, LinestringError>,
) -> Result<Vec<usize>, LinestringError> {
    let indices_a = indices_a?;
    let indices_b = indices_b?;

    if indices_a.is_empty() {
        return Ok(indices_b);
    }
    if indices_b.is_empty() {
        return Ok(indices_a);
    }

    let starting_index = VertexIndex {
        from_a: true,
        index: indices_a[0],
        index_index: 0,
    };

    let mut hull = Vec::with_capacity((indices_a.len() + indices_b.len()) / 4);
    // To track visited points, it contain index, not index_index
    //let mut visited = AHashSet::with_capacity(indices_a.len() + indices_b.len());
    let mut point_on_hull = starting_index.clone();
    /*println!(
        "starting point {:?}:{:?}",
        vertices[point_on_hull.index], point_on_hull
    );*/
    loop {
        /*println!(
            "loop: point_on_hull {:?}:{:?}",
            vertices[point_on_hull.index], point_on_hull
        );*/

        hull.push(point_on_hull.index);
        if point_on_hull.index != starting_index.index {
            // don't mark the starting_point or we won't know where to stop
            //let _ = visited.insert(point_on_hull.index);
        }
        let mut end_point = {
            if point_on_hull.from_a {
                let index_index = (point_on_hull.index_index + 1) % indices_a.len();
                let index = indices_a[index_index];
                if index != point_on_hull.index {
                    VertexIndex {
                        from_a: true,
                        index: indices_a[index_index],
                        index_index,
                    }
                } else {
                    // this is sus, this is just the first point from hull B
                    VertexIndex {
                        from_a: false,
                        index: indices_b[0],
                        index_index: 0,
                    }
                }
            } else {
                let index_index = (point_on_hull.index_index + 1) % indices_b.len();
                let index = indices_b[index_index];
                if index != point_on_hull.index {
                    VertexIndex {
                        from_a: false,
                        index: indices_b[index_index],
                        index_index,
                    }
                } else {
                    // this is sus, this is just the first point from hull A
                    VertexIndex {
                        from_a: true,
                        index: indices_a[0],
                        index_index: 0,
                    }
                }
            }
        };
        /*println!(
            "loop: end_point  {:?}:{:?}",
            vertices[end_point.index], end_point
        );*/

        if point_on_hull.from_a {
            // when we have found a good end_point from indices_b and we find a bad one, we can
            // exit the loop
            let mut found_a_legit_b_point = false;
            for (jj, j) in indices_b.iter().enumerate() {
                let j = *j;

                if j == point_on_hull.index || j == end_point.index
                /*|| visited.contains(&j)*/
                {
                    continue;
                }

                let orient = point_orientation(
                    vertices[point_on_hull.index],
                    vertices[end_point.index],
                    vertices[j],
                );
                if orient == Orientation::Right {
                    // replace end_point with the candidate if candidate is to the right,
                    end_point.index_index = jj;
                    end_point.index = j;
                    end_point.from_a = false;
                    found_a_legit_b_point = true;
                } else if orient == Orientation::Collinear {
                    let distance_sq_candidate =
                        vertices[point_on_hull.index].distance_sq(vertices[j]);
                    let distance_sq_end_point =
                        vertices[point_on_hull.index].distance_sq(vertices[end_point.index]);

                    if distance_sq_candidate > distance_sq_end_point {
                        // if point_on_hull->end_point && point_on_hull->candidate are collinear and
                        // the distance to the collinear is greater we also update end_point
                        end_point.index_index = jj;
                        end_point.index = j;
                        end_point.from_a = false;
                        found_a_legit_b_point = true;
                    } else if j != starting_index.index {
                        // This is a collinear point that is closer to the current point, mark it as visited
                        //let _ = visited.insert(j);
                    }
                } else {
                    // we should have a test here to see if we can end the loop.
                    // if we only knew that end_point was a legit point
                    if found_a_legit_b_point {
                        break;
                    }
                }
            }
        } else {
            // when we have found a good end_point from indices_a and we find a bad one, we can
            // exit the loop
            let mut found_a_legit_a_point = false;
            for (jj, j) in indices_a.iter().enumerate() {
                let j = *j;

                if j == point_on_hull.index || j == end_point.index
                /*|| visited.contains(&j)*/
                {
                    continue;
                }

                let orient = point_orientation(
                    vertices[point_on_hull.index],
                    vertices[end_point.index],
                    vertices[j],
                );
                if orient == Orientation::Right {
                    // replace end_point with the candidate if candidate is to the right,
                    end_point.index_index = jj;
                    end_point.index = j;
                    end_point.from_a = true;
                    found_a_legit_a_point = true;
                } else if orient == Orientation::Collinear {
                    let distance_sq_candidate =
                        vertices[point_on_hull.index].distance_sq(vertices[j]);
                    let distance_sq_end_point =
                        vertices[point_on_hull.index].distance_sq(vertices[end_point.index]);

                    if distance_sq_candidate > distance_sq_end_point {
                        // if point_on_hull->end_point && point_on_hull->candidate are collinear and
                        // the distance to the collinear is greater we also update end_point
                        end_point.index_index = jj;
                        end_point.index = j;
                        end_point.from_a = true;
                        found_a_legit_a_point = true;
                    } else if j != starting_index.index {
                        // This is a collinear point that is closer to the current point, mark it as visited
                        //let _ = visited.insert(j);
                    }
                } else {
                    // we should have a test here to see if we can end the loop.
                    // if we only knew that end_point was a legit point
                    if found_a_legit_a_point {
                        break;
                    }
                }
            }
        }
        /*println!(
            "end of loop: end_point  {:?}:{:?}",
            vertices[end_point.index], end_point
        );
        println!();
         */
        point_on_hull = end_point.clone();
        if end_point.index == starting_index.index {
            break;
        }
    }
    //println!("visited:{:?}", visited);
    /*println!("indexed_gift_wrap:");
    println!("starting_index {:?}", input_indices[starting_index]);
    println!("input {:?}", input_vertices);
    println!("input indices {:?}", input_indices);
    println!("result {:?}", hull);*/

    Ok(hull)
}

/// Finds the convex hull using a parallel version of Graham's scan, similar to Chan's
/// algorithm.
/// [Graham's Scan](https://en.wikipedia.org/wiki/Graham_scan)
/// [Chan's Algorithm](https://en.wikipedia.org/wiki/Chan%27s_algorithm)
///
/// # Arguments
///
/// * `vertices` - The input vertices.
/// * `indices` - The selected indices to the vertices. Not all vertices need to be mentioned here;
/// only those in the indices list are used by the function.
/// * `per_thread_chunk_size` - The size of the chunk that each thread should work on.
///
/// # Returns
///
/// A list of indices pointing back into the vertices list, representing the convex hull.
/// The list will be closed, i.e the first and last element will be the same.
/// # Errors
///
/// Returns an error if any errors occur during computation.E.g. if any vertex component is NaN or infinite
///
/// # Example
///
///```
/// # use linestring::linestring_2d::{LineString2,convex_hull};
/// # use vector_traits::glam::Vec2;
/// # use rand::{Rng, SeedableRng, rngs::StdRng};
/// let mut rng:StdRng = SeedableRng::from_seed([42; 32]);
/// let mut points = Vec::<Vec2>::new();
/// for _i in 0..1023 {
///   let p: [f32; 2] = [rng.gen_range(0.0..4096.0), rng.gen_range(0.0..4096.0)];
///   points.push(p.into());
/// }
///
/// let a = points;
/// let indices:Vec<usize> = (0..a.len()).collect();
/// let convex_hull = convex_hull::convex_hull_par(&a, &indices,250)?;
/// let center = Vec2{x:2000_f32, y:2000.0};
///
/// for point in convex_hull.iter() {
///   for edge in convex_hull.chunks_exact(2) {
///     // all segments should have the center point at the 'left' side
///     assert!(convex_hull::is_point_left(a[edge[0]], a[edge[1]], center));
///     // all points on the hull should be 'left' of every segment in the hull
///     assert!(convex_hull::is_point_left_allow_collinear(a[edge[0]], a[edge[1]], a[*point]));
///   }
/// }
/// # Ok::<(), linestring::LinestringError>(())
///```
pub fn convex_hull_par<T: GenericVector2>(
    vertices: &[T],
    mut indices: &[usize],
    per_thread_chunk_size: usize,
) -> Result<Vec<usize>, LinestringError> {
    if indices.len() <= 1 {
        return Ok(indices.to_vec());
    }
    if indices.first().unwrap() == indices.last().unwrap() {
        // disregard the duplicated loop point if it exists
        indices = &indices[0..indices.len() - 1];
    }
    if indices.len() <= 2 {
        return Ok(indices.to_vec());
    }
    if indices.len() == 3 {
        return if is_point_left(
            vertices[indices[0]],
            vertices[indices[1]],
            vertices[indices[2]],
        ) {
            // input_indices was already CCW
            Ok(indices.to_vec())
        } else {
            // Convert from CW to CCW
            Ok(vec![indices[0], indices[2], indices[1]])
        };
    }

    // Create chunks of indices
    let chunks: Vec<_> = indices.chunks(per_thread_chunk_size).collect();

    let start_index = {
        //println!("vertices:{:?}", vertices);
        let start_indexes = chunks
            .par_iter()
            .map(|chunk| {
                indexed_find_lowest_x(vertices, chunk).map(|(index_index, _)| chunk[index_index])
            })
            .collect::<Result<Vec<usize>, LinestringError>>()?;
        //println!("start_indexes:{:?}", start_indexes);

        start_indexes[indexed_find_lowest_x(vertices, &start_indexes)?.0]
    };
    //println!("start_index:{}", start_index);

    // Run indexed_graham_scan in parallel on each chunk
    let mut hull = chunks
        .into_par_iter()
        .map(|chunk| indexed_graham_scan_no_loop(vertices, chunk, None))
        // Combine the partial convex hulls using indexed_convex_hull_combiner
        .reduce(
            || Ok(vec![start_index]),
            |result, partial_hull| {
                match result {
                    Ok(indices) => combine_indexed_convex_hull(vertices, Ok(indices), partial_hull),
                    Err(some_err) => Err(some_err), // Pass through error
                }
            },
        )?;

    // complete the loop, `hull` now represents a closed loop of points.
    // Each edge can be iterated over by `rv.points.iter().window(2)`
    if hull.len() > 1 {
        hull.push(*hull.first().unwrap());
    }
    Ok(hull)
}

/// Returns true if the 'a' convex hull entirely contains the 'b' convex hull (inclusive)
/// This function is single threaded.
/// ```
/// # use linestring::linestring_2d::{LineString2,convex_hull};
/// # use rand::{Rng, SeedableRng, rngs::StdRng};
/// # use vector_traits::glam::Vec2;
///
/// let mut rng:StdRng = SeedableRng::from_seed([42; 32]);
/// let mut points = Vec::<Vec2>::new();
/// for _i in 0..4023 {
///    let p: [f32; 2] = [rng.gen_range(0.0..4096.0), rng.gen_range(0.0..4096.0)];
///    points.push(p.into());
/// }
///
/// let a = convex_hull::graham_scan(&points)?;
///
/// let mut points = Vec::<Vec2>::new();
/// for _i in 0..1023 {
///    let p: [f32; 2] = [rng.gen_range(1000.0..2000.0), rng.gen_range(1000.0..2000.0)];
///    points.push(p.into());
/// }
///
/// let b = convex_hull::graham_scan(&points)?;
///
/// assert!(convex_hull::contains_convex_hull(&a, &b));
/// assert!(!convex_hull::contains_convex_hull(&b, &a));
/// # Ok::<(), linestring::LinestringError>(())
///```
pub fn contains_convex_hull<T: GenericVector2>(a: &Vec<T>, b: &Vec<T>) -> bool {
    if a.point_count() <= 1 {
        return false;
    }
    if b.is_empty() {
        // Is 'nothing' contained inside any finite convex hull or not?
        return true;
    }

    // First, quickly check midpoint of 'a' against start and midpoint of 'b'
    let a_midpoint = a.len() / 2;
    let b_midpoint = b.len() / 2;

    let a_mid_edge = (a[a_midpoint - 1], a[a_midpoint]);
    if !is_point_left_allow_collinear(a_mid_edge.0, a_mid_edge.1, b[0])
        || !is_point_left_allow_collinear(a_mid_edge.0, a_mid_edge.1, b[b_midpoint])
    {
        return false;
    }

    // If both checks passed, now do a complete check
    b.iter().all(|p| contains_point_inclusive(a, *p))
}

/// Returns true if the 'a' convex hull entirely contains the 'b' convex hull (inclusive)
/// This function uses rayon for parallel iteration.
/// ```
/// # use linestring::linestring_2d::{LineString2,convex_hull};
/// # use rand::{Rng, SeedableRng, rngs::StdRng};
/// # use vector_traits::glam::Vec2;
///
/// let mut rng:StdRng = SeedableRng::from_seed([42; 32]);
/// let mut vertices = Vec::<Vec2>::new();
/// for _i in 0..4023 {
///    let p: [f32; 2] = [rng.gen_range(0.0..4096.0), rng.gen_range(0.0..4096.0)];
///    vertices.push(p.into());
/// }
/// // close the loop
/// vertices.push(*vertices.first().unwrap());
///
///
/// let a = convex_hull::graham_scan(&vertices)?;
///
/// let mut points = Vec::<Vec2>::new();
/// for _i in 0..1023 {
///    let p: [f32; 2] = [rng.gen_range(1000.0..2000.0), rng.gen_range(1000.0..2000.0)];
///    points.push(p.into());
/// }
/// // close the loop
/// points.push(*points.first().unwrap());
///
/// let b = convex_hull::graham_scan(&points)?;
///
/// assert!(convex_hull::contains_convex_hull_par(&a, &b));
/// assert!(!convex_hull::contains_convex_hull_par(&b, &a));
/// # Ok::<(), linestring::LinestringError>(())
///```
pub fn contains_convex_hull_par<T: GenericVector2>(a: &Vec<T>, b: &Vec<T>) -> bool {
    if a.point_count() <= 1 {
        return false;
    }
    if b.is_empty() {
        // Is 'nothing' contained inside any finite convex hull or not?
        return true;
    }
    b.par_iter().all(|p| contains_point_inclusive(a, *p))
}

/// Returns true if the 'a' convex hull contains the 'b' point (exclusive)
/// This function is single threaded.
///```
/// # use vector_traits::glam::DVec2;
/// # use linestring::linestring_2d::{convex_hull,LineString2};
///
/// let mut hull = Vec::<DVec2>::with_capacity(4);
/// hull.push([0.0,0.0].into());
/// hull.push([10.0,0.0].into());
/// hull.push([10.0,10.0].into());
/// hull.push([0.0,10.0].into());
/// hull.push([0.0,0.0].into());
///
/// assert!(!convex_hull::contains_point_exclusive(&hull, [5.0,0.0].into()));
/// assert!(!convex_hull::contains_point_exclusive(&hull, [0.0,0.0].into()));
/// assert!(!convex_hull::contains_point_exclusive(&hull, [10.0,10.0].into()));
/// assert!(convex_hull::contains_point_exclusive(&hull, [5.0,6.0].into()));
/// assert!(!convex_hull::contains_point_exclusive(&hull, [10.0000001,10.0].into()));
/// assert!(convex_hull::contains_point_exclusive(&hull, [9.99999,9.99999].into()));
/// assert!(!convex_hull::contains_point_exclusive(&hull, [10.0,9.99999].into()));
///
///```
pub fn contains_point_exclusive<T: GenericVector2>(input: &Vec<T>, p: T) -> bool {
    //println!("testing {:?}", p);
    match input.len() {
        0 => return false,
        1 => return p == input[0],
        2 => {
            if input[0] != input[1] {
                return false;
            }
        }
        _ => (),
    };
    let (mut slice, input_offset) = if input[0] == input[input.len() - 1] {
        // skip the explicit vertex loop, from now on the loop is implicit
        (&input[1..], 1)
    } else {
        (&input[..], 0)
    };
    let mut retain_index_0 = false;

    while (slice.len() > 3) || (retain_index_0 && slice.len() > 2) {
        let mid = if retain_index_0 {
            ((slice.len() + 1) / 2) - 1
        } else {
            slice.len() / 2
        };
        //println!("new mid:{mid} len:{}", slice.len());
        if !is_point_left_allow_collinear(slice[mid - 1], slice[mid], p) {
            //println!("return false 2");
            return false;
        }
        /*if retain_index_0 {
            println!("slicing edge: [0,{:?}]", slice[mid]);
        } else {
            println!("slicing edge: [{:?},{:?}]", slice[0], slice[mid]);
        }*/
        if (retain_index_0 && is_point_left_allow_collinear(input[input_offset], slice[mid], p))
            || (!retain_index_0 && is_point_left_allow_collinear(slice[0], slice[mid], p))
        {
            // Point is to the left of the second half
            slice = &slice[mid..];
            retain_index_0 = true;
            //println!("new (left) slice: 0+{:?}", slice);
        } else {
            // Point is to the right of the first half
            slice = &slice[..mid + 1];
            /*if retain_index_0 {
                println!("new (right) slice: 0+{:?}", slice);
            } else {
                println!("new (right) slice: {:?}", slice);
            }*/
        }
    }

    //println!("{:?}", slice );
    // Check the final slice (triangle)
    if retain_index_0 {
        if !is_point_left(input[input_offset], slice[0], p) {
            //println!("return false 3.1 for slice: 0+{:?}", slice);
            return false;
        }
        for window in slice.windows(2) {
            if !is_point_left(window[0], window[1], p) {
                //println!("return false 3.2 for slice: 0+{:?}", slice);
                return false;
            }
        }
        if !is_point_left(slice[slice.len() - 1], input[input_offset], p) {
            //println!("return false 3.3 slice:0+{:?}", slice);
            return false;
        }
    } else {
        for window in slice.windows(2) {
            //println!("testing {:?} {:?}", window[0], window[1]);
            if !is_point_left(window[0], window[1], p) {
                //println!("return false 3");
                return false;
            }
        }
        if !is_point_left(slice[slice.len() - 1], slice[0], p) {
            //println!("return false 4 slice:{:?}", slice);
            return false;
        }
    }
    true
}

/// Returns true if the 'a' convex hull contains the 'b' point (inclusive)
/// This function is single threaded.
///```
/// # use vector_traits::glam::DVec2;
/// # use linestring::linestring_2d::{convex_hull,LineString2};
///
/// let mut hull = Vec::<DVec2>::with_capacity(4);
/// hull.push([0.0,0.0].into());
/// hull.push([10.0,0.0].into());
/// hull.push([10.0,10.0].into());
/// hull.push([0.0,10.0].into());
/// hull.push([0.0,0.0].into());
///
/// assert!(convex_hull::contains_point_inclusive(&hull, [0.0,5.0].into()));
/// assert!(!convex_hull::contains_point_inclusive(&hull, [-0.000001,5.0].into()));
/// assert!(convex_hull::contains_point_inclusive(&hull, [0.0,0.0].into()));
/// assert!(convex_hull::contains_point_inclusive(&hull, [10.0,5.0].into()));
/// assert!(!convex_hull::contains_point_inclusive(&hull, [10.000001,5.0].into()));
/// assert!(convex_hull::contains_point_inclusive(&hull, [9.999,5.0].into()));
/// assert!(convex_hull::contains_point_inclusive(&hull, [10.0,5.0].into()));
/// assert!(!convex_hull::contains_point_inclusive(&hull, [10.0,10.000001].into()));
///```
pub fn contains_point_inclusive<T: GenericVector2>(input: &Vec<T>, p: T) -> bool {
    //println!("testing {:?}", p);

    match input.len() {
        0 => return false,
        1 => return p == input[0],
        2 => {
            if input[0] != input[1] {
                // this is no convex hull
                // TODO: but technically we should check if p lies on the line
                return false;
            }
        }
        _ => (),
    };

    let (mut slice, input_offset) = if input[0] == input[input.len() - 1] {
        // skip the explicit vertex loop, from now on the loop is implicit
        (&input[1..], 1)
    } else {
        (&input[..], 0)
    };
    let mut retain_index_0 = false;

    while (slice.len() > 3) || (retain_index_0 && slice.len() > 2) {
        let mid = if retain_index_0 {
            ((slice.len() + 1) / 2) - 1
        } else {
            slice.len() / 2
        };
        //println!("new mid:{mid} len:{}", slice.len());
        if !is_point_left_allow_collinear(slice[mid - 1], slice[mid], p) {
            //println!("return false 2");
            return false;
        }
        /*if retain_index_0 {
            println!("slicing edge: [0,{:?}]", slice[mid]);
        } else {
            println!("slicing edge: [{:?},{:?}]", slice[0], slice[mid]);
        }*/
        if (retain_index_0 && is_point_left_allow_collinear(input[input_offset], slice[mid], p))
            || (!retain_index_0 && is_point_left_allow_collinear(slice[0], slice[mid], p))
        {
            // Point is to the left of the second half
            slice = &slice[mid..];
            retain_index_0 = true;
            //println!("new (left) slice: 0+{:?}", slice);
        } else {
            // Point is to the right of the first half
            slice = &slice[..mid + 1];
            /*if retain_index_0 {
                println!("new (right) slice: 0+{:?}", slice);
            } else {
                println!("new (right) slice: {:?}", slice);
            }*/
        }
    }

    //println!("{:?}", slice );
    // Check the final slice (triangle)
    if retain_index_0 {
        if !is_point_left_allow_collinear(input[input_offset], slice[0], p) {
            //println!("return false 3.1 for slice: 0+{:?}", slice);
            return false;
        }
        for window in slice.windows(2) {
            if !is_point_left_allow_collinear(window[0], window[1], p) {
                //println!("return false 3.2 for slice: 0+{:?}", slice);
                return false;
            }
        }
        if !is_point_left_allow_collinear(slice[slice.len() - 1], input[input_offset], p) {
            //println!("return false 3.3 slice:0+{:?}", slice);
            return false;
        }
    } else {
        for window in slice.windows(2) {
            //println!("testing {:?} {:?}", window[0], window[1]);
            if !is_point_left_allow_collinear(window[0], window[1], p) {
                //println!("return false 3");
                return false;
            }
        }
        if !is_point_left_allow_collinear(slice[slice.len() - 1], slice[0], p) {
            //println!("return false 4 slice:{:?}", slice);
            return false;
        }
    }
    true
}
