// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2021,2023 lacklustr@protonmail.com https://github.com/eadf

// This file is part of the linestring crate.

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use linestring::linestring_2d::convex_hull;
use rand::{rngs::StdRng, Rng, SeedableRng};
use vector_traits::glam::Vec2;
const SIZE: usize = 10000;
const CHUNK_SIZE: usize = 1000;
#[cfg(test)]
fn convex_hull_graham_scan_bench(c: &mut Criterion) {
    let mut rng: StdRng = SeedableRng::from_seed([42; 32]);
    let mut points = Vec::<Vec2>::new();
    for _i in 0..SIZE {
        let p: [f32; 2] = [rng.gen_range(0.0..1.0), rng.gen_range(0.0..1.0)];
        points.push(p.into());
    }

    c.bench_function("convex_hull_graham_scan_bench", |b| {
        b.iter(|| {
            black_box({
                let _a = convex_hull::graham_scan(&points).unwrap();
            });
        })
    });
}

#[cfg(test)]
fn convex_hull_gift_wrap_bench(c: &mut Criterion) {
    let mut rng: StdRng = SeedableRng::from_seed([42; 32]);
    let mut points = Vec::<Vec2>::new();
    for _i in 0..SIZE {
        let p: [f32; 2] = [rng.gen_range(0.0..1.0), rng.gen_range(0.0..1.0)];
        points.push(p.into());
    }

    c.bench_function("convex_hull_gift_wrap_bench", |b| {
        b.iter(|| {
            black_box({
                let _a = convex_hull::gift_wrap(&points).unwrap();
            });
        })
    });
}

#[cfg(test)]
fn convex_hull_indexed_graham_scan_bench(c: &mut Criterion) {
    let mut rng: StdRng = SeedableRng::from_seed([42; 32]);
    let mut points = Vec::<Vec2>::new();
    for _i in 0..SIZE {
        let p: [f32; 2] = [rng.gen_range(0.0..1.0), rng.gen_range(0.0..1.0)];
        points.push(p.into());
    }
    let indices: Vec<usize> = (0..points.len()).collect();

    c.bench_function("convex_hull_indexed_graham_scan_bench", |b| {
        b.iter(|| {
            black_box({
                let _a = convex_hull::indexed_graham_scan(&points, &indices).unwrap();
            });
        })
    });
}

#[cfg(test)]
fn convex_hull_indexed_gift_wrap_bench(c: &mut Criterion) {
    let mut rng: StdRng = SeedableRng::from_seed([42; 32]);
    let mut points = Vec::<Vec2>::new();
    for _i in 0..SIZE {
        let p: [f32; 2] = [rng.gen_range(0.0..1.0), rng.gen_range(0.0..1.0)];
        points.push(p.into());
    }
    let indices: Vec<usize> = (0..points.len()).collect();

    c.bench_function("convex_hull_indexed_gift_wrap_bench", |b| {
        b.iter(|| {
            black_box({
                let _a = convex_hull::indexed_gift_wrap(&points, &indices).unwrap();
            });
        })
    });
}

#[cfg(test)]
fn convex_hull_convex_hull_par(c: &mut Criterion) {
    let mut rng: StdRng = SeedableRng::from_seed([42; 32]);
    let mut points = Vec::<Vec2>::new();
    for _i in 0..SIZE {
        let p: [f32; 2] = [rng.gen_range(0.0..1.0), rng.gen_range(0.0..1.0)];
        points.push(p.into());
    }
    let indices: Vec<usize> = (0..points.len()).collect();

    c.bench_function("convex_hull_convex_hull_par", |b| {
        b.iter(|| {
            black_box({
                let _a = convex_hull::convex_hull_par(&points, &indices, CHUNK_SIZE).unwrap();
            });
        })
    });
}

criterion_group!(
    convex_hull_benches,
    convex_hull_graham_scan_bench,
    convex_hull_indexed_graham_scan_bench,
    convex_hull_gift_wrap_bench,
    convex_hull_indexed_gift_wrap_bench,
    convex_hull_convex_hull_par,
);
criterion_main!(convex_hull_benches);
