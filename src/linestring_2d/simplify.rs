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

use crate::linestring_2d::{Line2, LineString2, PriorityDistance};
use itertools::Itertools;
use std::collections::BinaryHeap;
use vector_traits::{GenericScalar, GenericVector2};

/// The actual implementation of rdp
pub(super) fn simplify_rdp_recursive<T: GenericVector2>(
    rv: &mut Vec<T>,
    points: &[T],
    distance_predicate_sq: T::Scalar,
) {
    if points.len() <= 2 {
        rv.push(*points.last().unwrap());
        return;
    }

    let start_point = points[0];
    let end_point = points[points.len() - 1];

    let (max_dist_sq, index) = points
        .iter()
        .enumerate()
        .skip(1)
        .take(points.len() - 2)
        .map(|(i, &point)| {
            (
                super::distance_to_line_squared_safe(start_point, end_point, point),
                i,
            )
        })
        .max_by(|(dist1, _), (dist2, _)| {
            dist1
                .partial_cmp(dist2)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .unwrap_or((-T::Scalar::ONE, 0)); // default that's unlikely to be used

    if max_dist_sq <= distance_predicate_sq {
        rv.push(end_point);
    } else {
        simplify_rdp_recursive(rv, &points[..index + 1], distance_predicate_sq);
        simplify_rdp_recursive(rv, &points[index..], distance_predicate_sq);
    }
}

/// Simplify using Visvalingam–Whyatt algorithm. This algorithm will delete 'points_to_delete'
/// of points from the polyline with the smallest area defined by one point and it's neighbours.
pub(super) fn simplify_vw<T: GenericVector2>(line: &Vec<T>, points_to_delete: usize) -> Vec<T> {
    let connected = line.is_connected();
    if line.len() <= 2 {
        // Nothing to do here, we can't delete endpoints
        return line.clone();
    }
    // priority queue key: area, value: indices of line.points + a copy of area.
    // When a point is removed it's previously calculated area-to-neighbour value will not be
    // removed. Instead new areas will simply be added to the priority queue.
    // If a removed node is pop():ed it will be checked against the link_tree hash map.
    // If it is not in there or if the area doesn't match it will simply be ignored and a
    // new value pop():ed.

    let mut search_tree = BinaryHeap::<PriorityDistance<T>>::new();
    // map from node number to remaining neighbours of that node. All indices of line.points
    // AHashMap::<node_id:usize, (prev_node_id:usize, next_node_id:usize, area:T)>
    let mut link_tree = ahash::AHashMap::<usize, (usize, usize, T::Scalar)>::default();
    {
        let mut iter_i = line.iter().enumerate();
        let mut iter_j = line.iter().enumerate().skip(1);
        // the k iterator will terminate before i & j, so the iter_i & iter_j unwrap()s are safe
        for k in line.iter().enumerate().skip(2) {
            let i = iter_i.next().unwrap();
            let j = iter_j.next().unwrap();
            // define the area between point i, j & k as search criteria
            let area = Line2::triangle_area_squared_times_4(*i.1, *j.1, *k.1);
            search_tree.push(PriorityDistance {
                key: area,
                index: j.0,
            });
            // point j is connected to point i and k
            let _ = link_tree.insert(j.0, (i.0, k.0, area));
        }
    }
    if connected {
        // add an extra point at the end, faking the loop
        let i = line.len() - 2;
        let j = line.len() - 1;
        let k = line.len();
        let area = Line2::triangle_area_squared_times_4(line[i], line[j], line[0]);
        search_tree.push(PriorityDistance {
            key: area,
            index: j,
        });
        let _ = link_tree.insert(j, (i, k, area));
    }

    let line_points_len = line.len();

    let mut deleted_nodes: usize = 0;
    loop {
        if search_tree.is_empty() || link_tree.len() == 2 || deleted_nodes >= points_to_delete {
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
                            line[prev],
                            line[next % line_points_len],
                            line[next_next % line_points_len],
                        );
                        search_tree.push(PriorityDistance {
                            key: area,
                            index: next,
                        });
                        let _ = link_tree.insert(next, (prev, next_next, area));

                        let area = Line2::triangle_area_squared_times_4(
                            line[prev_prev],
                            line[prev],
                            line[next % line_points_len],
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
                    let area = Line2::triangle_area_squared_times_4(
                        line[prev_prev],
                        line[prev],
                        line[next % line_points_len],
                    );
                    search_tree.push(PriorityDistance {
                        key: area,
                        index: prev,
                    });
                    let _ = link_tree.insert(prev, (prev_prev, next, area));
                    continue;
                };

                if let Some(next_next) = next_next {
                    let area = Line2::triangle_area_squared_times_4(
                        line[prev],
                        line[next % line_points_len],
                        line[next_next % line_points_len],
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
    if !connected {
        [0_usize]
            .iter()
            .copied()
            .chain(
                link_tree
                    .keys()
                    .sorted_unstable()
                    .copied()
                    .chain([line.len() - 1].iter().copied()),
            )
            .map(|x| line[x])
            .collect()
    } else {
        [0_usize]
            .iter()
            .copied()
            .chain(link_tree.keys().sorted_unstable().copied())
            .map(|x| line[x])
            .collect()
    }
}
