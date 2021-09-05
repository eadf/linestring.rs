use criterion::{criterion_group, criterion_main, Criterion};
use linestring::linestring_2d::LineString2;

#[cfg(test)]
fn intersection_bench(c: &mut Criterion) {
    let mut vertices = Vec::<cgmath::Point2<f32>>::with_capacity(10000);
    let mut angle = 0.0_f32;
    let mut radius = 0.1_f32;
    for _i in 0..10000 {
        vertices.push(cgmath::Point2 {
            x: angle.cos() * radius,
            y: angle.sin() * radius,
        });
        angle += 0.1;
        radius += 0.2;
    }
    let linestring = LineString2::with_iter(vertices.iter());

    c.bench_function("intersection_bench", |b| {
        b.iter(|| {
            assert!(!linestring.is_self_intersecting().unwrap());
        })
    });
}

criterion_group!(benches1, intersection_bench);
criterion_main!(benches1);
