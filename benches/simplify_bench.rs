// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2021,2023 lacklustr@protonmail.com https://github.com/eadf

// This file is part of the linestring crate.

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use linestring::linestring_2d::LineString2;
use vector_traits::glam;

#[cfg(test)]
fn simplify_bench_rdp(c: &mut Criterion) {
    let mut linestring = Vec::<glam::Vec2>::with_capacity(10000);
    let mut angle = 0.0_f32;
    let mut radius = 0.1_f32;
    for _i in 0..12000 {
        linestring.push(glam::Vec2 {
            x: angle.cos() * radius,
            y: angle.sin() * radius,
        });
        angle += 0.1;
        radius += 0.2;
    }

    c.bench_function("simplify_bench_rdp", |b| {
        b.iter(|| {
            #[allow(clippy::unit_arg)]
            black_box({
                linestring.simplify_rdp(0.01);
            });
        })
    });
}

#[cfg(test)]
fn simplify_bench_vw(c: &mut Criterion) {
    let mut linestring = Vec::<glam::Vec2>::with_capacity(10000);
    let mut angle = 0.0_f32;
    let mut radius = 0.1_f32;
    for _i in 0..12000 {
        linestring.push(glam::Vec2 {
            x: angle.cos() * radius,
            y: angle.sin() * radius,
        });
        angle += 0.1;
        radius += 0.2;
    }

    c.bench_function("simplify_bench_vw", |b| {
        b.iter(|| {
            #[allow(clippy::unit_arg)]
            black_box({
                linestring.simplify_vw(11000);
            });
        })
    });
}

criterion_group!(simplify_benches, simplify_bench_rdp, simplify_bench_vw);
criterion_main!(simplify_benches);
