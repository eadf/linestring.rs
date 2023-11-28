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
use crate::linestring_3d::{Aabb3, ChunkIterator, Line3, PriorityDistance, WindowIterator};
use vector_traits::{approx::ulps_eq, GenericVector3};

impl<T: GenericVector3> Default for Aabb3<T> {
    fn default() -> Self {
        Self { min_max: None }
    }
}

impl<'a, T: GenericVector3> Iterator for WindowIterator<'a, T> {
    type Item = Line3<T>;

    fn next(&mut self) -> Option<Self::Item> {
        self.0.next().map(|window| Line3 {
            start: window[0],
            end: window[1],
        })
    }
}

impl<'a, T: GenericVector3> Iterator for ChunkIterator<'a, T> {
    type Item = Line3<T>;

    fn next(&mut self) -> Option<Self::Item> {
        self.0.next().map(|window| Line3 {
            start: window[0],
            end: window[1],
        })
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
