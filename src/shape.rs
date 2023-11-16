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

use ahash::AHashMap;
use smallvec::SmallVec;
use vob::Vob;

/// a Vob that counts how many bits it sets
pub(crate) struct SetVob {
    number_of_set_bits: usize,
    vob: Vob<u32>,
}

impl SetVob {
    pub(crate) fn fill_with_false(size: usize) -> Self {
        let mut v: Vob<u32> = Vob::<u32>::new_with_storage_type(0);
        v.resize(size, false);
        Self {
            number_of_set_bits: 0,
            vob: v,
        }
    }

    #[inline(always)]
    pub(crate) fn get(&self, index: usize) -> bool {
        self.vob.get(index).unwrap()
    }

    #[inline(always)]
    pub(crate) fn set(&mut self, index: usize) {
        if self.vob.set(index, true) {
            self.number_of_set_bits += 1;
        }
    }

    #[inline(always)]
    pub(crate) fn number_of_set_bits(&self) -> usize {
        self.number_of_set_bits
    }
}

/// Follows a detected line starting at `current`. If `next` is Some, that vertex id should be picked next.
/// `adjacency_map`: contains a map by vertex id key, and a list of adjacent vertices as `value`
/// `termination_nodes`: a set of nodes where lines should end. Such vertices has no, or more than two neighbors.
/// `visited`: is a set of nodes we have already visited, but only non-termination vertexes are
/// marked because termination nodes needs to be used several times.
/// returns a list of vertex id:s constituting the detected line in .windows(2) format
fn unwind_line(
    mut current: usize,
    next: Option<usize>,
    adjacency_map: &AHashMap<usize, SmallVec<[usize; 2]>>,
    termination_nodes: &SetVob,
    visited: &mut SetVob,
) -> Vec<usize> {
    let mut line = Vec::<usize>::default();
    let mut prev = Option::<usize>::None;
    //println!("unwind_line: current:{} next:{:?}", current, next);
    if let Some(next) = next {
        /*assert!(
            adjacency_map.get(&current).unwrap().contains(&next),
            "current:{} array:{:?} does not contain {}",
            current,
            adjacency_map.get(&current).unwrap(),
            next
        );*/

        //println!("pushed to line:{}", current);
        line.push(current);
        if !termination_nodes.get(current) {
            // don't mark termination nodes
            //println!("visited pushed {}", current);
            visited.set(current);
        }
        prev = Some(current);
        current = next;
    }
    loop {
        if visited.get(current) || termination_nodes.get(current) {
            // we have gone round a closed shape
            /*println!(
                "detected visited (or termination) vertex:{} group:{:?}",
                current, line
            );*/
            //println!("pushed to line:{}", current);
            line.push(current);
            break;
        }
        //println!("visited pushed {}", current);
        visited.set(current);
        //println!("pushed to line:{}", current);
        line.push(current);
        if let Some(neighbours) = adjacency_map.get(&current) {
            /*println!(
                "current:{}, neighbours:{:?} visited0:{}, visited1:{}",
                current,
                neighbours,
                visited.contains(&neighbours[0]),
                visited.contains(&neighbours[1])
            );*/
            if neighbours.len() != 2 {
                break;
            }

            if !(visited.get(neighbours[0]) || prev.is_some() && prev.unwrap() == neighbours[0]) {
                /*println!(
                    "neighbours[1]={} was visited:{}",
                    &neighbours[1],
                    visited.contains(&neighbours[1])
                );
                println!("picking current = {}", neighbours[0]); //assert!(visited.contains(&neighbours[0]));
                */
                // neighbour 0 is unvisited
                prev = Some(current);
                current = neighbours[0];
            } else if !(visited.get(neighbours[1])
                || prev.is_some() && prev.unwrap() == neighbours[1])
            {
                /*println!(
                    "neighbours[0]={} was visited:{}",
                    &neighbours[0],
                    visited.contains(&neighbours[0])
                );
                println!("picking current = {}", neighbours[1]);
                */

                //assert!(!visited.contains(&neighbours[1]));
                // neighbour 1 is unvisited
                prev = Some(current);
                current = neighbours[1]
            } else {
                //println!("nowhere to go");
                break;
            }
        } else {
            break;
        }
    }
    //println!("Found a line:{:?}", line);
    line
}

/// calculate the highest index and the adjacency map
fn adjacency_map(indices: &[usize]) -> (usize, AHashMap<usize, SmallVec<[usize; 2]>>) {
    let mut adjacency_map = AHashMap::<usize, SmallVec<[usize; 2]>>::with_capacity(indices.len());
    let mut max_index = 0;
    indices.chunks_exact(2).for_each(|chunk| {
        let i0 = chunk[0];
        let i1 = chunk[1];
        max_index = max_index.max(i0).max(i1);
        let entry = adjacency_map.entry(i0).or_default();
        if !entry.contains(&i1) {
            entry.push(i1);
        }

        let entry = adjacency_map.entry(i1).or_default();
        if !entry.contains(&i0) {
            entry.push(i0);
        }
    });
    (max_index, adjacency_map)
}

/// generate the termination and candidate lists
#[allow(clippy::type_complexity)]
fn termination_candidate_nodes(
    max_index: usize,
    adjacency_map: &AHashMap<usize, SmallVec<[usize; 2]>>,
) -> (SetVob, Vec<(usize, SmallVec<[usize; 2]>)>) {
    // these vertices are connected to 0 or >2 other vertices
    let mut termination_nodes = SetVob::fill_with_false(max_index + 1);

    // these vertices are also connected to 0 or >2 other vertices, but will be continuously used/pop:ed.
    let mut candidate_nodes = Vec::<(usize, SmallVec<[usize; 2]>)>::default();

    // Build the candidates and termination_nodes set
    for v_id in 0..max_index + 1 {
        if let Some(neighbours) = adjacency_map.get(&v_id) {
            if neighbours.len() != 2 {
                termination_nodes.set(v_id);
                candidate_nodes.push((v_id, neighbours.clone()));
            }
        } else {
            // vertices not even mentioned in the indices list
            termination_nodes.set(v_id)
        }
    }
    /*assert_eq!(
        termination_nodes_len,
        termination_nodes.iter_set_bits(..).count()
    );*/
    (termination_nodes, candidate_nodes)
}

/// Divides the `indices` into continuous shapes of vertex indices, removing duplicated edges.
///
/// # Examples
///
/// ```
/// use linestring::prelude::divide_into_shapes;
/// let indices = vec![0, 1, 2, 3, 1, 4, 4, 5, 6, 7];
/// let result = divide_into_shapes(&indices);
/// assert_eq!(result.len(), 3);
/// assert_eq!(result[0], vec![6, 7]);
/// assert_eq!(result[1], vec![5, 4, 1, 0]);
/// assert_eq!(result[2], vec![2, 3]);
/// ```
///
/// `indices` is a list of unordered vertex indices in the `.chunks_exact(2)` format. For example, [0, 1, 2, 3, 1, 4, 4, 5, 6, 7]
/// represents edges at [0, 1], [2, 3], [1, 4], [4, 5], [6, 7]. The function returns lists of lists of continuous connected shapes
/// in the `.windows(2)` format. If a shape describes a loop, the first and last index will be the same.
///
/// Duplicated edges will be removed during processing.
///
/// # Parameters
///
/// - `indices`: A slice of unordered vertex indices in the `.chunks(2)` format.
///
/// # Returns
///
/// A `Vec<Vec<usize>>` containing lists of lists of continuous connected shapes.
///
pub fn divide_into_shapes(indices: &[usize]) -> Vec<Vec<usize>> {
    // a Vec containing identified shapes, and those are vertex indices in .windows(2) format
    let mut group_container = Vec::<Vec<usize>>::new();
    // a map for vertex id to a list of adjacent vertices
    let (max_index, adjacency_map) = adjacency_map(indices);

    // these vertices are connected to 0 or >2 other vertices
    let (termination_nodes, mut candidate_nodes) =
        termination_candidate_nodes(max_index, &adjacency_map);

    // vertices that has already been marked as used
    let mut visited = SetVob::fill_with_false(indices.len());
    /*
    println!(
        "adjacency_map:{:?}",
        adjacency_map
            .iter()
            .sorted_unstable_by(|a, b| a.0.cmp(b.0))
            .collect::<Vec<_>>()
    );
    println!("termination_nodes:{:?}", termination_nodes);
    println!("max_index:{:?}", max_index);
    println!("candidate_nodes:{:?}", candidate_nodes);
    */
    let mut current: usize = 0;

    // first stage: pop from the candidate list
    while !candidate_nodes.is_empty() {
        let mut next_vertex = Option::<usize>::None;
        'outer: while !candidate_nodes.is_empty() && next_vertex.is_none() {
            if let Some((candidate, ref mut array)) = candidate_nodes.last_mut() {
                current = *candidate;
                //println!("current:{}, array:{:?}", current, array);
                while !array.is_empty() {
                    let n_vertex = array.pop().unwrap();
                    if termination_nodes.get(n_vertex) {
                        if current < n_vertex {
                            // only add termination node -> termination node connections once
                            group_container.push(vec![current, n_vertex]);
                            //println!("1group_container:{:?}", group_container);
                        }
                    } else if visited.get(n_vertex) {
                        continue;
                    } else {
                        next_vertex = Some(n_vertex);
                        break 'outer;
                    }
                }
            }
            if let Some((_, a)) = candidate_nodes.pop() {
                assert!(a.is_empty())
            }
        }
        if next_vertex.is_some() {
            // next_vertex should now contain something
            group_container.push(unwind_line(
                current,
                next_vertex,
                &adjacency_map,
                &termination_nodes,
                &mut visited,
            ));
            //println!("2group_container:{:?}", group_container);
        }
    }
    /*
    println!(
        "stage two: visited:{} termination_nodes:{} total:{} max_index:{}",
        visited.len(),
        termination_nodes.len(),
        visited.len() + termination_nodes.len(),
        max_index
    );
    for group in &group_container {
        println!("group:{:?}", group);
    }
    println!(
        "visited:{:?}, len:{}",
        visited
            .iter()
            .sorted_unstable_by(|a, b| a.cmp(b))
            .collect::<Vec<_>>(),
        visited.len()
    );
    println!(
        "termination_nodes:{:?}, len:{}",
        termination_nodes
            .iter()
            .sorted_unstable_by(|a, b| a.cmp(b))
            .collect::<Vec<_>>(),
        termination_nodes.len()
    );
    assert!(candidate_nodes.is_empty());
    */
    // second stage, only loops remaining
    if visited.number_of_set_bits() + termination_nodes.number_of_set_bits() < max_index + 1 {
        let mut min_index = 0_usize;
        'outer: loop {
            current = min_index;
            while visited.get(current) || termination_nodes.get(current) {
                current += 1;
                min_index = current;
                if current >= max_index {
                    // we probably have detected a loop again
                    current = min_index;
                    break;
                }
            }
            if current > max_index {
                if visited.number_of_set_bits() + termination_nodes.number_of_set_bits()
                    >= max_index
                {
                    break 'outer;
                }
                // we did not find any isolated vertex, just pick one and start from there
                current = min_index;
                //only_loops_remain = true;
            }
            // `current` should now point to a vertex only connected to one, or more than 2 other vertexes
            // it could also point to a two-way connected vertex, but then all the rest are loops
            assert!(!visited.get(current));
            {
                // unravel the line at `current`
                let mut line = unwind_line(
                    current,
                    None,
                    &adjacency_map,
                    &termination_nodes,
                    &mut visited,
                );
                if line.len() > 1 {
                    line.push(*line.first().unwrap());
                    group_container.push(line);
                }
            }
            // current is now at the end or at a junction
            if visited.number_of_set_bits() + termination_nodes.number_of_set_bits() >= max_index {
                break;
            }
        }
    }
    group_container
}
