// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2021,2023 lacklustr@protonmail.com https://github.com/eadf

// This file is part of the linestring crate.

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use linestring::linestring_2d::LineString2;
use vector_traits::glam::Vec2;

#[cfg(test)]
fn intersection_bench_1(c: &mut Criterion) {
    let mut linestring = Vec::<Vec2>::with_capacity(10000);
    let mut angle = 0.0_f32;
    let mut radius = 0.1_f32;
    for _i in 0..10000 {
        linestring.push(Vec2 {
            x: angle.cos() * radius,
            y: angle.sin() * radius,
        });
        angle += 0.1;
        radius += 0.2;
    }

    c.bench_function("intersection_bench", |b| {
        b.iter(|| {
            #[allow(clippy::unit_arg)]
            black_box({
                assert!(!linestring.is_self_intersecting().unwrap());
            });
        })
    });
}

criterion_group!(intersection_bench, intersection_bench_1);
criterion_main!(intersection_bench);
