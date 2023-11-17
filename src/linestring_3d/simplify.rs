use crate::linestring_3d::{Line3, PriorityDistance};
use ahash::AHashMap;
use itertools::Itertools;
use std::collections::BinaryHeap;
use vector_traits::{GenericScalar, GenericVector3};

/// Simplify using Ramer–Douglas–Peucker algorithm adapted for 3d
pub(super) fn simplify_rdp<T: GenericVector3>(line: &[T], distance_predicate: T::Scalar) -> Vec<T> {
    if line.len() <= 2 {
        return line.to_vec();
    }

    let mut rv: Vec<T> = Vec::with_capacity(line.len());
    rv.push(line[0]);
    simplify_rdp_recursive(&mut rv, line, distance_predicate * distance_predicate);
    rv
}

/// The actual implementation of rdp
fn simplify_rdp_recursive<T: GenericVector3>(
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

/// Simplify using Visvalingam–Whyatt algorithm adapted for 3d.
/// This algorithm will delete 'points_to_delete' of points from the polyline with the smallest
/// area defined by one point and it's neighbours.
pub(super) fn simplify_vw<T: GenericVector3>(line: &[T], points_to_delete: usize) -> Vec<T> {
    if line.len() <= 2 {
        // Nothing to do here, we can't delete endpoints
        return line.to_vec();
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
    let mut link_tree = AHashMap::<usize, (usize, usize, T::Scalar)>::default();
    {
        let mut iter_i = line.iter().enumerate();
        let mut iter_j = line.iter().enumerate().skip(1);
        // the k iterator will terminate before i & j, so the iter_i & iter_j unwrap()s are safe
        for k in line.iter().enumerate().skip(2) {
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

    let line_points_len = line.len();

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
                            &line[prev],
                            &line[next % line_points_len],
                            &line[next_next % line_points_len],
                        );
                        search_tree.push(PriorityDistance {
                            key: area,
                            index: next,
                        });
                        let _ = link_tree.insert(next, (prev, next_next, area));

                        let area = Line3::triangle_area_squared_times_4(
                            &line[prev_prev],
                            &line[prev],
                            &line[next % line_points_len],
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
                        &line[prev_prev],
                        &line[prev],
                        &line[next % line_points_len],
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
                        &line[prev],
                        &line[next % line_points_len],
                        &line[next_next % line_points_len],
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

    [0_usize]
        .iter()
        .copied()
        .chain(link_tree.keys().sorted_unstable().copied())
        .map(|x| line[x])
        .collect()
}

/// Simplify a polyline using the Ramer–Douglas–Peucker algorithm.
///
/// The Ramer–Douglas–Peucker algorithm simplifies a polyline by recursively removing points
/// that are within a specified distance of the line segment connecting the start and end points.
/// This can be useful for reducing the number of points in a polyline while preserving its shape.
///
/// # Parameters
///
/// - `vertices`: A slice of generic vectors representing the points in the polyline.
/// - `indices`: An index slice representing the indices of the points in the polyline.
/// - `distance_predicate`: The squared distance threshold. Points within this squared distance
///   of the line segment will be removed during simplification.
///
/// # Returns
///
/// A `Vec<usize>` containing the indices of the simplified polyline.
///
/// # Examples
///
/// ```
/// # use vector_traits::glam;
/// use linestring::prelude::indexed_simplify_rdp_3d;
///
/// let line: Vec<glam::Vec3> = vec![
///     (0.0, 3.0, 0.0).into(),
///     (1.0, 2.0, 0.0).into(),
///     (4.0, 1.0, 0.0).into(),
///     (0.0, 0.0, 0.0).into(),
/// ];
/// let indices: Vec<usize> = vec![0, 1, 2, 3];
/// // Simplify the polyline using the Ramer–Douglas–Peucker algorithm
/// let simplified_line = indexed_simplify_rdp_3d(&line, &indices, 1.0);
/// // The result will have fewer points while preserving the overall shape
/// assert_eq!(2, simplified_line.windows(2).len());
/// ```
///
/// In this example, the original polyline `line` is simplified using the Ramer–Douglas–Peucker
/// algorithm with a squared distance threshold of 1.0. The resulting simplified polyline is then
/// checked for the expected number of points using `windows(2)`.
///
/// For more information on the Ramer–Douglas–Peucker algorithm, see:
/// [Ramer–Douglas–Peucker algorithm](https://en.wikipedia.org/wiki/Ramer–Douglas–Peucker_algorithm)
pub fn indexed_simplify_rdp<T: GenericVector3>(
    vertices: &[T],
    indices: &[usize],
    distance_predicate: T::Scalar,
) -> Vec<usize> {
    if vertices.len() <= 2 {
        return indices.to_vec();
    }

    let mut simplified_indices = Vec::new();
    simplified_indices.push(indices[0]);

    indexed_simplify_rdp_recursive(
        &mut simplified_indices,
        vertices,
        indices,
        distance_predicate * distance_predicate,
    );

    simplified_indices
}

/// The actual implementation of the indexed rdp
// todo: this code is identical to the 2d version, can it be consolidated?
fn indexed_simplify_rdp_recursive<T: GenericVector3>(
    simplified_indices: &mut Vec<usize>,
    vertices: &[T],
    indices: &[usize],
    distance_predicate_sq: T::Scalar,
) {
    //println!(">indices:{:?} len:{}", indices, indices.len());
    if indices.len() <= 2 {
        simplified_indices.push(*indices.last().unwrap());
        return;
    }

    let start_index = indices[0];
    let end_index = indices[indices.len() - 1];

    let (max_dist_sq, index_index) = indices
        .iter()
        .enumerate()
        .skip(1)
        .take(indices.len() - 2)
        .map(|(i, &point_i)| {
            //assert_eq!(point_i, indices[i]);
            (
                super::distance_to_line_squared_safe(
                    vertices[start_index],
                    vertices[end_index],
                    vertices[point_i],
                ),
                i,
            )
        })
        .max_by(|(dist1, _), (dist2, _)| {
            dist1
                .partial_cmp(dist2)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .unwrap_or((-T::Scalar::ONE, 0)); // default that's unlikely to be used
                                          //println!("max dist:{:?}, index_index:{}, simplified_indices:{:?}", max_dist_sq, index_index, simplified_indices);
    if max_dist_sq <= distance_predicate_sq {
        simplified_indices.push(end_index);
    } else {
        indexed_simplify_rdp_recursive(
            simplified_indices,
            vertices,
            &indices[..=index_index],
            distance_predicate_sq,
        );
        indexed_simplify_rdp_recursive(
            simplified_indices,
            vertices,
            &indices[index_index..],
            distance_predicate_sq,
        );
    }
    //println!("<");
}
