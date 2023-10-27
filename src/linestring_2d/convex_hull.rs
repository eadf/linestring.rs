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
#[cfg(test)]
mod tests;

use crate::linestring_2d::LineString2;
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

/// finds the point with lowest x
fn find_lowest_x<T: GenericVector2>(points: &[T]) -> (usize, T) {
    points
        .iter()
        .enumerate()
        .min_by(|&(_, a), &(_, b)| {
            a.x()
                .partial_cmp(&b.x())
                .unwrap_or(Ordering::Equal)
                .then_with(|| a.y().partial_cmp(&b.y()).unwrap())
        })
        .map(|(i, v)| (i, *v))
        .unwrap()
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
///```
/// # use linestring::linestring_2d;
/// # use linestring::linestring_2d::convex_hull;
/// # use vector_traits::glam::Vec2;
/// # use rand::{Rng, SeedableRng, rngs::StdRng};
/// let mut rng:StdRng = SeedableRng::from_seed([42; 32]);
/// let mut points = Vec::<Vec2>::new();
/// for _i in 0..50 {
///   let p: [f32; 2] = [rng.gen_range(0.0..4096.0), rng.gen_range(0.0..4096.0)];
///   points.push(p.into());
/// }
///
/// let a = linestring_2d::LineString2::with_vec(points);
/// let convex_hull = convex_hull::gift_wrap(&a.points());
/// let center = Vec2{x:2000_f32, y:2000.0};
///
/// for p in convex_hull.points().iter() {
///   for l in convex_hull.iter() {
///     // all segments should have the center point at the 'left' side
///     assert!(convex_hull::point_orientation(l.start, l.end, center).is_left());
///     assert!(convex_hull::is_point_left(l.start, l.end, center));
///     // all points on the hull should be 'left' of every segment in the hull
///     assert!(convex_hull::point_orientation(l.start, l.end, *p).is_left_or_collinear());
///   }
/// }
///```
pub fn gift_wrap<T: GenericVector2>(mut input_points: &[T]) -> LineString2<T> {
    if !input_points.is_empty() && (input_points[0] == input_points[input_points.len() - 1]) {
        // disregard the duplicated loop point if it exists
        input_points = &input_points[1..];
    }
    if input_points.len() <= 2 {
        return LineString2::with_vec(input_points.to_vec());
    }
    if input_points.len() == 3 {
        return if is_point_left(input_points[0], input_points[1], input_points[2]) {
            LineString2::with_vec(input_points.to_vec())
        } else {
            LineString2::with_vec(vec![input_points[0], input_points[2], input_points[1]])
        };
    }

    let starting_point = find_lowest_x(input_points).0;

    let mut hull = LineString2::with_capacity(input_points.len());
    let mut point_on_hull = starting_point;
    //let mut already_on_hull = crate::util::VobU32::fill_with_false(input_points.len());

    loop {
        hull.0.push(input_points[point_on_hull]);
        //if point_on_hull != starting_point {
        //    // don't mark the starting_point or we won't know where to stop
        //    already_on_hull.set_true(point_on_hull);
        //}
        let mut end_point = (point_on_hull + 1) % input_points.len();

        for j in 0..input_points.len() {
            if
            /*already_on_hull.get_or_false(j) || */
            j == point_on_hull {
                continue;
            }

            let orient = point_orientation(
                input_points[point_on_hull],
                input_points[end_point],
                input_points[j],
            );
            // todo: keep a set of discarded co-linear candidates
            if orient == Orientation::Right
                || orient == Orientation::Collinear
                    && input_points[point_on_hull].distance_sq(input_points[j])
                        > input_points[point_on_hull].distance_sq(input_points[end_point])
            {
                // replace end_point with the candidate if candidate is to the right,
                // or if point_on_hull->end_point && point_on_hull->candidate are collinear and
                // the distance to the collinear is greater.
                end_point = j;
            }
        }

        point_on_hull = end_point;
        if end_point == starting_point {
            // complete the loop, `hull` now represents a closed loop of points.
            // Each edge can be iterated over by `hull.iter().window(2)`
            hull.0.push(input_points[end_point]);
            break;
        }
    }

    hull
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
/// let a = LineString2::with_vec(points);
/// #[allow(deprecated)]
/// let convex_hull = convex_hull::graham_scan_wo_atan2(&a.points());
/// let center = Vec2{x:2000_f32, y:2000.0};
///
/// for p in convex_hull.points().iter() {
///   for l in convex_hull.iter() {
///     // all segments should have the center point at the 'left' side
///     assert!(convex_hull::is_point_left(l.start, l.end, center));
///     // all points on the hull should be 'left' of every segment in the hull
///     assert!(convex_hull::is_point_left_allow_collinear(l.start, l.end, *p));
///   }
/// }
///```
#[deprecated(since = "0.9.0", note = "please use `graham_scan` instead")]
pub fn graham_scan_wo_atan2<T: GenericVector2>(mut input_points: &[T]) -> LineString2<T> {
    if !input_points.is_empty() && (input_points[0] == input_points[input_points.len() - 1]) {
        // disregard the duplicated loop point if it exists
        input_points = &input_points[1..];
    }
    if input_points.len() <= 2 {
        return LineString2::with_vec(input_points.to_vec());
    }
    if input_points.len() == 3 {
        return if is_point_left(input_points[0], input_points[1], input_points[2]) {
            LineString2::with_vec(input_points.to_vec())
        } else {
            LineString2::with_vec(vec![input_points[0], input_points[2], input_points[1]])
        };
    }

    let mut input_points = input_points.to_vec();

    let (_starting_index, starting_point) = find_lowest_x(&input_points);

    let comparator = |b: &T, c: &T| {
        let ccr = cross_2d(starting_point, *b, *c);
        match T::Scalar::ZERO.partial_cmp(&ccr) {
            Some(Ordering::Greater) => Ordering::Greater, //CCR
            Some(Ordering::Less) => Ordering::Less,       //CR
            Some(Ordering::Equal) => {
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

    let mut rv = LineString2::with_capacity(input_points.len());
    rv.0.push(starting_point);

    for p in input_points.iter() {
        while rv.0.len() > 1 {
            let last = rv.0.len() - 1;
            if T::Scalar::ZERO >= cross_2d(rv.0[last - 1], rv.0[last], *p) {
                let _ = rv.0.pop();
            } else {
                break;
            }
        }
        if rv.0.last().unwrap() != p {
            rv.0.push(*p);
        }
    }

    if !rv.is_connected() {
        let first = rv.0.first().unwrap();
        rv.0.push(*first)
    };
    rv
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
/// let a = LineString2::with_vec(points);
/// let convex_hull = convex_hull::graham_scan(&a.points());
/// let center = Vec2{x:2000_f32, y:2000.0};
///
/// for p in convex_hull.points().iter() {
///   for l in convex_hull.iter() {
///     // all segments should have the center point at the 'left' side
///     assert!(convex_hull::is_point_left(l.start, l.end, center));
///     // all points on the hull should be 'left' of every segment in the hull
///     assert!(convex_hull::is_point_left_allow_collinear(l.start, l.end, *p));
///   }
/// }
///```
pub fn graham_scan<T: GenericVector2>(mut input_points: &[T]) -> LineString2<T> {
    if !input_points.is_empty() && (input_points[0] == input_points[input_points.len() - 1]) {
        // disregard the duplicated loop point if it exists
        input_points = &input_points[1..];
    }
    if input_points.len() <= 2 {
        return LineString2::with_vec(input_points.to_vec());
    }
    if input_points.len() == 3 {
        return if is_point_left(input_points[0], input_points[1], input_points[2]) {
            LineString2::with_vec(input_points.to_vec())
        } else {
            LineString2::with_vec(vec![input_points[0], input_points[2], input_points[1]])
        };
    }

    let (_, starting_point) = find_lowest_x(input_points);
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
    let mut rv = Vec::<T>::with_capacity(input_points.len());
    rv.push(starting_point);

    for (p, _, _) in input_points.iter() {
        if *p == starting_point {
            continue;
        }
        while rv.len() > 1 {
            let last = rv.len() - 1;
            let cross = cross_2d(rv[last - 1], rv[last], *p);
            if T::Scalar::ZERO >= cross {
                //println!("popping last: {:?}, cross:{:.4}", rv[last], cross);
                let _ = rv.pop();
            } else {
                //println!("break for {:?}", p);
                break;
            }
        }
        //println!("pushing p:{:?}", p);
        rv.push(*p);
    }
    //println!("result {:?}", rv);

    // complete the loop, `hull` now represents a closed loop of points.
    // Each edge can be iterated over by `rv.points.iter().window(2)`
    let first = rv.first().unwrap();
    rv.push(*first);
    LineString2::with_vec(rv)
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
/// let a = LineString2::with_vec(points);
/// let a = convex_hull::graham_scan(&a.points());
///
/// let mut points = Vec::<Vec2>::new();
/// for _i in 0..1023 {
///    let p: [f32; 2] = [rng.gen_range(1000.0..2000.0), rng.gen_range(1000.0..2000.0)];
///    points.push(p.into());
/// }
///
/// let b = LineString2::with_vec(points);
/// let b = convex_hull::graham_scan(&b.points());
///
/// assert!(convex_hull::contains_convex_hull(&a, &b));
/// assert!(!convex_hull::contains_convex_hull(&b, &a));
///```
pub fn contains_convex_hull<T: GenericVector2>(a: &LineString2<T>, b: &LineString2<T>) -> bool {
    if a.point_count() <= 1 {
        return false;
    }
    if b.is_empty() {
        // Is 'nothing' contained inside any finite convex hull or not?
        return true;
    }

    // First, quickly check midpoint of 'a' against start and midpoint of 'b'
    let a_midpoint = a.0.len() / 2;
    let b_midpoint = b.0.len() / 2;

    let a_mid_edge = (a.0[a_midpoint - 1], a.0[a_midpoint]);
    if !is_point_left_allow_collinear(a_mid_edge.0, a_mid_edge.1, b.0[0])
        || !is_point_left_allow_collinear(a_mid_edge.0, a_mid_edge.1, b.0[b_midpoint])
    {
        return false;
    }

    // If both checks passed, now do a complete check
    let rv = b.0.iter().all(|p| contains_point_inclusive(a, *p));
    println!("rv:{rv}");
    rv
}

/// Returns true if the 'a' convex hull entirely contains the 'b' convex hull (inclusive)
/// This function uses rayon for parallel iteration.
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
/// // close the loop
/// points.push(*points.first().unwrap());
///
///
/// let a = LineString2::with_vec(points);
/// let a = convex_hull::graham_scan(&a.points());
///
/// let mut points = Vec::<Vec2>::new();
/// for _i in 0..1023 {
///    let p: [f32; 2] = [rng.gen_range(1000.0..2000.0), rng.gen_range(1000.0..2000.0)];
///    points.push(p.into());
/// }
/// // close the loop
/// points.push(*points.first().unwrap());
///
/// let b = LineString2::with_vec(points);
/// let b = convex_hull::graham_scan(&b.points());
///
/// assert!(convex_hull::contains_convex_hull_par(&a, &b));
/// assert!(!convex_hull::contains_convex_hull_par(&b, &a));
///```
pub fn contains_convex_hull_par<T: GenericVector2>(a: &LineString2<T>, b: &LineString2<T>) -> bool {
    if a.point_count() <= 1 {
        return false;
    }
    if b.is_empty() {
        // Is 'nothing' contained inside any finite convex hull or not?
        return true;
    }
    b.0.par_iter().all(|p| contains_point_inclusive(a, *p))
}

/// Returns true if the 'a' convex hull contains the 'b' point (exclusive)
/// This function is single threaded.
///```
/// # use vector_traits::glam::DVec2;
/// # use linestring::linestring_2d::{convex_hull,LineString2};
///
/// let mut hull = LineString2::<DVec2>::with_capacity(4);
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
pub fn contains_point_exclusive<T: GenericVector2>(input: &LineString2<T>, p: T) -> bool {
    //println!("testing {:?}", p);
    match input.0.len() {
        0 => return false,
        1 => return p == input.0[0],
        2 => {
            if input.0[0] != input.0[1] {
                return false;
            }
        }
        _ => (),
    };
    let (mut slice, input_offset) = if input.0[0] == input.0[input.0.len() - 1] {
        // skip the explicit vertex loop, from now on the loop is implicit
        (&input.0[1..], 1)
    } else {
        (&input.0[..], 0)
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
        if (retain_index_0 && is_point_left_allow_collinear(input.0[input_offset], slice[mid], p))
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
        if !is_point_left(input.0[input_offset], slice[0], p) {
            //println!("return false 3.1 for slice: 0+{:?}", slice);
            return false;
        }
        for window in slice.windows(2) {
            if !is_point_left(window[0], window[1], p) {
                //println!("return false 3.2 for slice: 0+{:?}", slice);
                return false;
            }
        }
        if !is_point_left(slice[slice.len() - 1], input.0[input_offset], p) {
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
/// let mut hull = LineString2::<DVec2>::with_capacity(4);
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
pub fn contains_point_inclusive<T: GenericVector2>(input: &LineString2<T>, p: T) -> bool {
    //println!("testing {:?}", p);

    match input.0.len() {
        0 => return false,
        1 => return p == input.0[0],
        2 => {
            if input.0[0] != input.0[1] {
                // this is no convex hull
                // TODO: but technically we should check if p lies on the line
                return false;
            }
        }
        _ => (),
    };

    let (mut slice, input_offset) = if input.0[0] == input.0[input.0.len() - 1] {
        // skip the explicit vertex loop, from now on the loop is implicit
        (&input.0[1..], 1)
    } else {
        (&input.0[..], 0)
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
        if (retain_index_0 && is_point_left_allow_collinear(input.0[input_offset], slice[mid], p))
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
        if !is_point_left_allow_collinear(input.0[input_offset], slice[0], p) {
            //println!("return false 3.1 for slice: 0+{:?}", slice);
            return false;
        }
        for window in slice.windows(2) {
            if !is_point_left_allow_collinear(window[0], window[1], p) {
                //println!("return false 3.2 for slice: 0+{:?}", slice);
                return false;
            }
        }
        if !is_point_left_allow_collinear(slice[slice.len() - 1], input.0[input_offset], p) {
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

/*pub fn contains_point_inclusive_works<T: GenericVector2>(input: &LineString2<T>, p: T) -> bool {
    println!("testing {:?}", p);

    match input.0.len() {
        0 => return false,
        1 => return p == input.0[0],
        2 => {
            if input.0[0] != input.0[1] {
                // this is no convex hull
                // TODO: but technically we should check if p lies on the line
                return false;
            }
        }
        _ => (),
    };
    // skip the explicit vertex loop, from now on the loop is implicit
    let vec_indices: Vec<u32> = input
        .0
        .iter()
        .skip(1)
        .enumerate()
        .map(|(i, _)| i as u32)
        .collect();
    let mut slice = &vec_indices[..];
    let mut retain_index_0 = false;

    while (slice.len() > 3) || (retain_index_0 && slice.len() > 2) {
        let mid = if retain_index_0 {
            ((slice.len() + 1) / 2) - 1
        } else {
            slice.len() / 2
        };
        println!("new mid:{mid} len:{}", slice.len());
        if !is_point_left_allow_collinear(
            input.0[slice[mid - 1] as usize],
            input.0[slice[mid] as usize],
            p,
        ) {
            println!("return false 2");
            return false;
        }
        if retain_index_0 {
            println!("slicing edge: [0,{:?}]", slice[mid]);
        } else {
            println!("slicing edge: [{:?},{:?}]", slice[0], slice[mid]);
        }
        if (retain_index_0
            && is_point_left_allow_collinear(input.0[0], input.0[slice[mid] as usize], p))
            || (!retain_index_0
                && is_point_left_allow_collinear(
                    input.0[slice[0] as usize],
                    input.0[slice[mid] as usize],
                    p,
                ))
        {
            // Point is to the left of the second half
            slice = &slice[mid..];
            retain_index_0 = true;
            println!("new (left) slice: 0+{:?}", slice);
        } else {
            // Point is to the right of the first half
            slice = &slice[..mid + 1];
            if retain_index_0 {
                println!("new (right) slice: 0+{:?}", slice);
            } else {
                println!("new (right) slice: {:?}", slice);
            }
        }
    }

    //println!("{:?}", slice );
    // Check the final slice (triangle)
    if retain_index_0 {
        if !is_point_left_allow_collinear(input.0[0], input.0[slice[0] as usize], p) {
            println!("return false 3.1 for slice: 0+{:?}", slice);
            return false;
        }
        for window in slice.windows(2) {
            if !is_point_left_allow_collinear(
                input.0[window[0] as usize],
                input.0[window[1] as usize],
                p,
            ) {
                println!("return false 3.2 for slice: 0+{:?}", slice);
                return false;
            }
        }
        if !is_point_left_allow_collinear(input.0[slice[slice.len() - 1] as usize], input.0[0], p) {
            println!("return false 3.3 slice:0+{:?}", slice);
            return false;
        }
    } else {
        for window in slice.windows(2) {
            //println!("testing {:?} {:?}", window[0], window[1]);
            if !is_point_left_allow_collinear(
                input.0[window[0] as usize],
                input.0[window[1] as usize],
                p,
            ) {
                println!("return false 3");
                return false;
            }
        }
        if !is_point_left_allow_collinear(
            input.0[slice[slice.len() - 1] as usize],
            input.0[slice[0] as usize],
            p,
        ) {
            println!("return false 4 slice:{:?}", slice);
            return false;
        }
    }

    true
}*/
