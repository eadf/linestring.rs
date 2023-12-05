// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2021,2023 lacklustr@protonmail.com https://github.com/eadf

// This file is part of the linestring crate.

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use itertools::Itertools;
use linestring::linestring_2d::indexed_intersection::IntersectionTester;
use vector_traits::glam::Vec2;

#[cfg(test)]
fn indexed_intersection_bench_1(c: &mut Criterion) {
    let mut vertices = Vec::<Vec2>::with_capacity(10000);
    let mut angle = 0.0_f32;
    let mut radius = 0.1_f32;
    for _i in 0..10000 {
        vertices.push(Vec2 {
            x: angle.cos() * radius,
            y: angle.sin() * radius,
        });
        angle += 0.1;
        radius += 0.2;
    }
    let edges: Vec<(usize, usize)> = (0..vertices.len()).tuples().collect();

    c.bench_function("indexed_intersection_bench", |b| {
        b.iter(|| {
            #[allow(clippy::unit_arg)]
            black_box({
                let (_output_vertices, _result_iter) = IntersectionTester::new(vertices.clone())
                    .with_ignore_end_point_intersections(true)
                    .unwrap()
                    .with_stop_at_first_intersection(true)
                    .unwrap()
                    .with_edges(edges.clone().iter())
                    .unwrap()
                    .compute()
                    .unwrap();
            });
        })
    });
}

criterion_group!(indexed_intersection_bench, indexed_intersection_bench_1);
criterion_main!(indexed_intersection_bench);
