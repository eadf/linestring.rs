use criterion::{criterion_group, criterion_main, Criterion};
use linestring::linestring_2d::LineString2;

#[cfg(test)]
fn simplify_bench_rdp(c: &mut Criterion) {
    let mut vertices = Vec::<cgmath::Point2<f32>>::with_capacity(10000);
    let mut angle = 0.0_f32;
    let mut radius = 0.1_f32;
    for _i in 0..12000 {
        vertices.push(cgmath::Point2 {
            x: angle.cos() * radius,
            y: angle.sin() * radius,
        });
        angle += 0.1;
        radius += 0.2;
    }
    let linestring = LineString2::with_iter(vertices.iter());

    c.bench_function("simplify_bench_rdp", |b| {
        b.iter(|| {
            linestring.simplify(0.01);
        })
    });
}

#[cfg(test)]
fn simplify_bench_vw(c: &mut Criterion) {
    let mut vertices = Vec::<cgmath::Point2<f32>>::with_capacity(10000);
    let mut angle = 0.0_f32;
    let mut radius = 0.1_f32;
    for _i in 0..12000 {
        vertices.push(cgmath::Point2 {
            x: angle.cos() * radius,
            y: angle.sin() * radius,
        });
        angle += 0.1;
        radius += 0.2;
    }
    let linestring = LineString2::with_iter(vertices.iter());

    c.bench_function("simplify_bench_vw", |b| {
        b.iter(|| {
            linestring.simplify_vw(11000);
        })
    });
}

criterion_group!(benches2, simplify_bench_rdp, simplify_bench_vw);
criterion_main!(benches2);
