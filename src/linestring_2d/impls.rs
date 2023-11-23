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
//! A module containing boiler-plate implementations of standard traits such as Default, From etc etc

use super::{
    Aabb2, ChunkIterator, Intersection, Line2, PriorityDistance, SimpleAffine, WindowIterator,
};
use std::{
    cmp::Ordering,
    fmt,
    fmt::{Debug, Display},
};
use vector_traits::{approx::*, num_traits::real::Real, GenericScalar, GenericVector2};

fn format_float<G>(value: G) -> String
where
    G: Real + Display,
{
    if value.fract() == G::zero() {
        format!("{:.1}", value)
    } else {
        format!("{}", value)
    }
}

impl<T: GenericVector2> PartialEq for Line2<T> {
    fn eq(&self, other: &Self) -> bool {
        ulps_eq!(self.start.x(), other.start.x())
            && ulps_eq!(self.start.y(), other.start.y())
            && ulps_eq!(self.end.x(), other.end.x())
            && ulps_eq!(self.end.y(), other.end.y())
    }
}
impl<T: GenericVector2> Eq for Line2<T> {}

impl<T: GenericVector2> PartialOrd for PriorityDistance<T> {
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<T: GenericVector2> Ord for PriorityDistance<T> {
    #[inline(always)]
    fn cmp(&self, other: &Self) -> Ordering {
        self.key.partial_cmp(&other.key).unwrap().reverse()
    }
}

impl<T: GenericVector2> PartialEq for PriorityDistance<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        ulps_eq!(self.key, other.key)
    }
}
impl<T: GenericVector2> Eq for PriorityDistance<T> {}

impl<T: GenericVector2> From<[T::Scalar; 4]> for Line2<T> {
    fn from(array: [T::Scalar; 4]) -> Self {
        Line2 {
            start: T::new_2d(array[0], array[1]),
            end: T::new_2d(array[2], array[3]),
        }
    }
}

impl<T: GenericVector2> From<[[T::Scalar; 2]; 2]> for Line2<T> {
    fn from(array: [[T::Scalar; 2]; 2]) -> Self {
        Line2 {
            start: T::new_2d(array[0][0], array[0][1]),
            end: T::new_2d(array[1][0], array[1][1]),
        }
    }
}

impl<T: GenericVector2> From<Aabb2<T>> for Vec<T> {
    /// creates a connected LineString2 from the outlines of the Aabb2
    fn from(other: Aabb2<T>) -> Self {
        if let Some(min_max) = other.min_max {
            let points = vec![
                min_max.0,
                T::new_2d(min_max.1.x(), min_max.0.y()),
                min_max.1,
                T::new_2d(min_max.0.x(), min_max.1.y()),
                min_max.0,
            ];
            points
        } else {
            Vec::<T>::default()
        }
    }
}

impl<T: GenericVector2> PartialEq for Aabb2<T> {
    fn eq(&self, other: &Self) -> bool {
        match (self.min_max, other.min_max) {
            (Some(slf), Some(other)) => {
                ulps_eq!(slf.0.x(), other.0.x())
                    && ulps_eq!(slf.1.x(), other.1.x())
                    && ulps_eq!(slf.0.y(), other.0.y())
                    && ulps_eq!(slf.1.y(), other.1.y())
            }
            (None, None) => true,
            _ => false,
        }
    }
}
impl<T: GenericVector2> Eq for Aabb2<T> {}

impl<T: GenericVector2> Debug for Intersection<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Overlap(a) => Debug::fmt(&a, f),
            Self::Intersection(a) => Debug::fmt(&a, f),
        }
    }
}

impl<T: GenericVector2> Default for SimpleAffine<T> {
    #[inline]
    fn default() -> Self {
        Self {
            a_offset: [T::Scalar::ZERO, T::Scalar::ZERO],
            scale: [T::Scalar::ONE, T::Scalar::ONE],
            b_offset: [T::Scalar::ZERO, T::Scalar::ZERO],
        }
    }
}

impl<'a, T: GenericVector2> Iterator for WindowIterator<'a, T> {
    type Item = Line2<T>;

    fn next(&mut self) -> Option<Self::Item> {
        self.0.next().map(|window| Line2 {
            start: window[0],
            end: window[1],
        })
    }
}

impl<'a, T: GenericVector2> Iterator for ChunkIterator<'a, T> {
    type Item = Line2<T>;

    fn next(&mut self) -> Option<Self::Item> {
        self.0.next().map(|window| Line2 {
            start: window[0],
            end: window[1],
        })
    }
}

impl<T: GenericVector2> ExactSizeIterator for WindowIterator<'_, T> {
    fn len(&self) -> usize {
        self.len()
    }
}

impl<T: GenericVector2> Debug for Line2<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "({},{})-({},{})",
            format_float(self.start.x()),
            format_float(self.start.y()),
            format_float(self.end.x()),
            format_float(self.end.y()),
        )
    }
}

impl<T: GenericVector2> AbsDiffEq for Line2<T> {
    type Epsilon = T::Scalar;

    fn default_epsilon() -> Self::Epsilon {
        T::Scalar::default_epsilon()
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        if T::Scalar::abs_diff_eq(&self.start.x(), &other.start.x(), epsilon)
            && T::Scalar::abs_diff_eq(&self.start.y(), &other.start.y(), epsilon)
        {
            T::Scalar::abs_diff_eq(&self.end.x(), &other.end.x(), epsilon)
                && T::Scalar::abs_diff_eq(&self.end.y(), &other.end.y(), epsilon)
        } else {
            T::Scalar::abs_diff_eq(&self.start.x(), &other.end.x(), epsilon)
                && T::Scalar::abs_diff_eq(&self.start.y(), &other.end.y(), epsilon)
                && T::Scalar::abs_diff_eq(&self.end.x(), &other.start.x(), epsilon)
                && T::Scalar::abs_diff_eq(&self.end.y(), &other.start.y(), epsilon)
        }
    }
}

impl<T: GenericVector2> UlpsEq for Line2<T> {
    fn default_max_ulps() -> u32 {
        T::Scalar::default_max_ulps()
    }

    fn ulps_eq(&self, other: &Self, epsilon: Self::Epsilon, max_ulps: u32) -> bool {
        if T::Scalar::ulps_eq(&self.start.x(), &other.start.x(), epsilon, max_ulps)
            && T::Scalar::ulps_eq(&self.start.y(), &other.start.y(), epsilon, max_ulps)
        {
            T::Scalar::ulps_eq(&self.end.x(), &other.end.x(), epsilon, max_ulps)
                && T::Scalar::ulps_eq(&self.end.y(), &other.end.y(), epsilon, max_ulps)
        } else {
            T::Scalar::ulps_eq(&self.start.x(), &other.end.x(), epsilon, max_ulps)
                && T::Scalar::ulps_eq(&self.start.y(), &other.end.y(), epsilon, max_ulps)
                && T::Scalar::ulps_eq(&self.end.x(), &other.start.x(), epsilon, max_ulps)
                && T::Scalar::ulps_eq(&self.end.y(), &other.start.y(), epsilon, max_ulps)
        }
    }
}

impl<T: GenericVector2> Default for Aabb2<T> {
    fn default() -> Self {
        Self { min_max: None }
    }
}

impl<'a, T: GenericVector2 + 'a, I: IntoIterator<Item = &'a T>> From<I> for Aabb2<T> {
    fn from(iter: I) -> Self {
        let mut aabb = Aabb2::default();
        for p in iter {
            aabb.update_with_point(*p);
        }
        aabb
    }
}
