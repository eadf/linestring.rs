// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2021,2023 lacklustr@protonmail.com https://github.com/eadf

// This file is part of the linestring crate.

#![deny(warnings)]
// Emit a compilation error if the required features are not set
#[cfg(not(all(feature = "glam", feature = "cgmath")))]
compile_error!("All of the traits 'glam' and 'cgmath' features must be enabled for tests");

use linestring::{
    linestring_2d::{self, Aabb2, Intersection, Line2, LineString2, SimpleAffine},
    linestring_3d::{save_to_obj_file, Line3, LineString3},
    prelude::{indexed_simplify_rdp_2d, indexed_simplify_rdp_3d},
    LinestringError,
};
use std::ops::Neg;
use vector_traits::{
    approx::{ulps_eq, AbsDiffEq, UlpsEq},
    glam::{dvec2, vec2, Vec2, Vec3, Vec3A},
    Approx, HasXY,
};

#[allow(dead_code)]
const EPSILON: f64 = 1e-10; // or some small value that's appropriate for your use case

fn silly_assert_approx_eq<T: SillyApproxEq + std::fmt::Debug>(v1: T, v2: T, epsilon: f64) {
    assert!(v1.silly_approx_eq(&v2, epsilon), "{:?} != {:?}", v1, v2);
}
trait SillyApproxEq {
    fn silly_approx_eq(&self, other: &Self, epsilon: f64) -> bool;
}

impl SillyApproxEq for f32 {
    fn silly_approx_eq(&self, other: &Self, epsilon: f64) -> bool {
        (self - other).abs() <= epsilon as f32
    }
}
impl SillyApproxEq for f64 {
    fn silly_approx_eq(&self, other: &Self, epsilon: f64) -> bool {
        (self - other).abs() <= epsilon
    }
}

impl SillyApproxEq for Vec2 {
    fn silly_approx_eq(&self, other: &Self, epsilon: f64) -> bool {
        (self.x - other.x).abs() <= epsilon as f32 && (self.y - other.y).abs() <= epsilon as f32
    }
}

impl SillyApproxEq for Vec3 {
    fn silly_approx_eq(&self, other: &Self, epsilon: f64) -> bool {
        (self.x - other.x).abs() <= epsilon as f32
            && (self.y - other.y).abs() <= epsilon as f32
            && (self.z - other.z).abs() <= epsilon as f32
    }
}

fn almost_equal<T: UlpsEq + std::fmt::Debug>(x1: T, x2: T, y1: T, y2: T) {
    assert!(ulps_eq!(&x1, &x2), "{:?}!={:?}", (x1, y1), (x2, y2));
    assert!(ulps_eq!(&y1, &y2), "{:?}!={:?}", (x1, y1), (x2, y2));
}

/// draws a line pivoting around (x,y) with 'angle' in degrees
/// l1 & l2 are lengths
fn pivot(x: f32, y: f32, l1: f32, l2: f32, angle: f32) -> linestring_2d::Line2<Vec2> {
    let l = linestring_2d::Line2 {
        start: Vec2 {
            x: x + angle.to_radians().cos() * l1,
            y: y + angle.to_radians().sin() * l1,
        },
        end: Vec2 {
            x: x + (angle + 180.0).to_radians().cos() * l2,
            y: y + (angle + 180.0).to_radians().sin() * l2,
        },
    };
    // nudge the result so that it really pivots at the (x,y) point
    let v = l.end - l.start;
    let v1 = -v * (l1 / (l1 + l2));
    let v2 = v * (l2 / (l1 + l2));

    linestring_2d::Line2 {
        start: Vec2 {
            x: x + v1.x,
            y: y + v1.y,
        },
        end: Vec2 {
            x: x + v2.x,
            y: y + v2.y,
        },
    }
}

#[test]
fn linestring2_1() {
    let points: Vec<[f32; 2]> = vec![[0., 0.], [1., 1.], [2., 2.], [3., 3.], [1., 10.]];
    let points_len = points.len();

    let linestring: Vec<Vec2> = points.into_iter().map(|v| v.into()).collect();
    assert_eq!(linestring.point_count(), points_len);
    assert_eq!(linestring.window_iter().len(), points_len - 1);
}

#[test]
fn linestring3_1() {
    let linestring: Vec<Vec3> = vec![
        [0_f32, 0., 0.].into(),
        [1., 1., 0.].into(),
        [2., 2., 0.].into(),
        [3., 3., 0.].into(),
        [1., 10., 0.].into(),
    ];

    #[allow(deprecated)]
    {
        assert_eq!(linestring.window_iter().len(), linestring.len() - 1);
    }
}

#[test]
fn line2_1() {
    let line = linestring_2d::Line2::<Vec2>::from([[10., 0.], [0., 11.]]);
    assert_eq!(line.start, Vec2::from([10., 0.]));
    assert_eq!(line.end, Vec2::from([0., 11.]));
}

#[test]
fn line3_1() {
    let line = Line3::<Vec3>::from([[10., 0., 9.], [0., 11., 9.]]);
    assert_eq!(line.start, Vec3::from([10., 0., 9.]));
    assert_eq!(line.end, Vec3::from([0., 11., 9.]));
}

#[test]
fn aabb_1() {
    let line = vec![vec2(0.0, 0.0), vec2(10.0, 1.0)];
    let aabb0 = Aabb2::from(line.iter());
    let aabb1 = Aabb2::from(line.iter());
    assert_eq!(aabb0, aabb1);
    let line2: Vec<Vec2> = aabb0.convex_hull().unwrap();
    assert_eq!(line2.len(), 5);
    assert_eq!(line2[0], line2[4]);
    let line3 = Vec::from(aabb1);
    assert_eq!(line3.len(), 5);
    assert_eq!(line3[0], line3[4]);
    for line in line3.window_iter() {
        assert!(line.abs_diff_eq(&line, f32::EPSILON));
        assert!(line.ulps_eq(&line, f32::EPSILON, f32::default_max_ulps()));
        assert_eq!(line, line);
    }
    let empty = Vec::<Vec2>::default();
    assert!(empty.is_empty());
    let default_afffine = SimpleAffine::<Vec2>::default();
    assert_eq!(default_afffine.a_offset[0], 0.0);
    assert_eq!(default_afffine.a_offset[1], 0.0);
    assert_eq!(default_afffine.scale[0], 1.0);
    assert_eq!(default_afffine.scale[1], 1.0);
    assert_eq!(default_afffine.b_offset[0], 0.0);
    assert_eq!(default_afffine.b_offset[0], 0.0);
}

#[test]
fn intersection_1() {
    let l1 = linestring_2d::Line2::<Vec2>::from([200., 200., 300., 300.]);
    let l2 = linestring_2d::Line2::<Vec2>::from([400., 200., 300., 300.]);
    let rv = l1.intersection_point(l2).unwrap().single();
    almost_equal(rv.x, 300.0, rv.y, 300.0);
}

#[test]
fn ray_intersection_1() {
    let l1 = vec![[0., 200.].into(), [0., -200.].into()];
    let l2 = linestring_2d::Line2::<Vec2>::from([-400., 0., 1., 0.]);
    let mut ray = l2.end - l2.start;
    let rv = l1.closest_ray_intersection(ray, l2.start).unwrap();
    almost_equal(rv.x, 0.0, rv.y, 0.0);
    let l1 = vec![[0., 200.].into(), [0., -200.].into()];
    let rv = l1.closest_ray_intersection(ray, l2.start).unwrap();
    almost_equal(rv.x, 0.0, rv.y, 0.0);
    ray = ray.neg();
    assert!(l1.closest_ray_intersection(ray, l2.start).is_none());
}
#[test]
fn ray_intersection_2() {
    let ray_origin: vector_traits::glam::Vec2 = (10., 200.).into();
    let ray: Vec2 = (1.0, 0.0).into();
    let pivot_p = Vec2::new(200.0, 200.0);
    // overlapping lines
    for r in (1..361).step_by(3) {
        //println!("r:{}", r);
        let line1 = pivot(pivot_p.x, pivot_p.y, 50.0, 4.0, r as f32);
        //println!("line1:{:?}", line1);
        let l1 = vec![line1.start, line1.end];
        let rv = l1.closest_ray_intersection(ray, ray_origin).unwrap();
        //println!("rv:{:?} l1:{:?}", rv, pivot_p);
        silly_assert_approx_eq(rv, pivot_p, 0.001);
        //silly_assert_approx_eq(rv, 200.0,  0.001);
    }
}

#[test]
fn intersection_2() {
    let l1 = linestring_2d::Line2::<Vec2>::from([200., 200., 300., 400.]);
    let l2 = linestring_2d::Line2::<Vec2>::from([400., 200., 300., 400.]);
    let rv = l1.intersection_point(l2).unwrap().single();
    almost_equal(rv.x, 300.0, rv.y, 400.0);
}

#[test]
fn intersection_3() {
    // line to point detection
    let l1 = linestring_2d::Line2::<Vec2>::from([200., 200., 300., 300.]);
    let l2 = linestring_2d::Line2::<Vec2>::from([250., 250., 250., 250.]);
    let rv = l1.intersection_point(l2).unwrap().single();
    almost_equal(rv.x, 250.0, rv.y, 250.0);
}

#[test]
fn intersection_4() {
    // line to point detection
    let l1 = linestring_2d::Line2::<Vec2>::from([300., 300., 200., 200.]);
    let l2 = linestring_2d::Line2::<Vec2>::from([250., 250., 250., 250.]);
    let rv = l1.intersection_point(l2).unwrap().single();
    almost_equal(rv.x, 250.0, rv.y, 250.0);
}

#[test]
fn intersection_5() {
    // line to point detection
    for r in (0..360).step_by(5) {
        let line = pivot(100.0, 100.0, 50.0, 50.0, r as f32);
        let vector = line.end - line.start;
        let point = linestring_2d::scale_to_coordinate(line.start, vector, 0.45);
        //println!("line:{:?}", line);
        //println!("point:{:?}", point);
        let rv = line
            .intersection_point(linestring_2d::Line2::new(point, point))
            .unwrap()
            .single();
        //println!("rv:{:?}", rv);
        //println!();
        almost_equal(rv.x, point.x, rv.y, point.y);
    }
}

#[test]
fn intersection_6() {
    let line1 = linestring_2d::Line2::new(Vec2 { x: 100.0, y: 150.0 }, Vec2 { x: 150.0, y: 100.0 });
    let line2 = linestring_2d::Line2::new(Vec2 { x: 150.0, y: 100.0 }, Vec2 { x: 160.0, y: 150.0 });
    let _rv = line1.intersection_point(line2);
}

#[test]
fn intersection_7() {
    // README.md example
    let line1 = linestring_2d::Line2::new(Vec2 { x: 100.0, y: 150.0 }, Vec2 { x: 150.0, y: 100.0 });
    let line2 = linestring_2d::Line2::new(Vec2 { x: 100.0, y: 150.0 }, Vec2 { x: 150.0, y: 100.0 });
    let _rv = line1.intersection_point(line2);
    match _rv {
        Some(Intersection::Intersection(_a)) => panic!("expected an overlap"),
        Some(Intersection::Overlap(_a)) => (),
        None => panic!("expected an overlap"),
    }
}

#[test]
fn intersection_9() {
    // overlapping lines
    for r in (0..360).step_by(3) {
        let is = pivot(200.0, 200.0, 50.0, 1.0, r as f32);
        /*
        println!(
            "line1:{:?} slope:{}",
            line1,
            (line1.start.x - line1.end.x) / (line1.start.y - line1.end.y)
        );*/

        let rv = is.intersection_point(is);
        match rv {
            Some(Intersection::<Vec2>::Intersection(_a)) => {
                panic!("expected an overlap, got {:?}", _a)
            }
            Some(Intersection::<Vec2>::Overlap(_a)) => {
                assert!(ulps_eq!(_a, is), "{:?}!={:?}", _a, is);
            }
            _ => panic!("expected an overlap, got None"),
        }
    }
}

#[test]
fn intersection_10() {
    // overlapping lines
    for r in (0..360).step_by(3) {
        let line1 = pivot(200.0, 200.0, 50.0, 1.0, r as f32);
        let line2 = linestring_2d::Line2 {
            start: line1.end,
            end: line1.start,
        };

        let rv = line1.intersection_point(line2);
        match rv {
            Some(Intersection::Intersection(_a)) => {
                panic!("expected an overlap, got {:?}", _a)
            }
            Some(Intersection::Overlap(_a)) => {
                assert!(ulps_eq!(_a, line1), "{:?}!={:?}", _a, line1);
            }
            _ => panic!("expected an overlap, got None"),
        }
    }
}

#[test]
fn intersection_12() {
    let l1 = Line2::<Vec2>::new((-1.0, 0.0).into(), (1000.0, 0.0).into());
    let l2 = Line2::<Vec2>::new((0.0, 0.0).into(), (5.0, 0.0).into());
    let rv = l1.intersection_point(l2);
    match rv {
        Some(Intersection::Intersection(_a)) => {
            panic!("expected an overlap, got {:?}", _a)
        }
        Some(Intersection::Overlap(l3)) => {
            assert!(ulps_eq!(l3, l2), "{:?}!={:?}", l3, l2);
        }
        _ => panic!("expected an overlap, got None"),
    }
    let rv = l2.intersection_point(l1);
    match rv {
        Some(Intersection::Intersection(_a)) => {
            panic!("expected an overlap, got {:?}", _a)
        }
        Some(Intersection::Overlap(l3)) => {
            assert!(ulps_eq!(l3, l2), "{:?}!={:?}", l3, l2);
        }
        _ => panic!("expected an overlap, got None"),
    }
}

#[test]
fn intersection_11() {
    let l1 = linestring_2d::Line2::<Vec2>::from([4.0, -9.0, -10.0, 2.0]);
    let l2 = linestring_2d::Line2::<Vec2>::from([-3.0, -3.0, -3.0, 3.0]);
    assert!(l1.intersection_point(l2).is_none());
    assert!(l2.intersection_point(l1).is_none());
}

#[test]
fn simplify_1() {
    let linestring: Vec<Vec2> = vec![[10f32, 10.], [13.0, 11.0], [20.0, 10.0]]
        .into_iter()
        .map(|v| v.into())
        .collect();
    assert_eq!(1, linestring.simplify_rdp(10.0).window_iter().len());
    println!("{:?}", linestring.simplify_rdp(0.1));
    assert_eq!(2, linestring.simplify_rdp(0.1).window_iter().len());
    let linestring: Vec<Vec2> = vec![[10f32, 10.], [20.0, 10.0]]
        .into_iter()
        .map(|v| v.into())
        .collect();
    assert_eq!(1, linestring.simplify_rdp(0.1).window_iter().len());
    let linestring: Vec<Vec2> = vec![[10f32, 10.], [15.0, 12.0], [17.0, 13.0], [20.0, 10.0]]
        .into_iter()
        .map(|v| v.into())
        .collect();
    //
    assert_eq!(2, linestring.simplify_rdp(1.1).window_iter().len());

    let line = vec![
        [77f32, 613.],
        [689., 650.],
        [710., 467.],
        [220., 200.],
        [120., 300.],
        [100., 100.],
        [77f32, 613.],
    ];
    let line: Vec<Vec2> = line.into_iter().map(|v| v.into()).collect();
    let result = line.simplify_rdp(1.0);
    assert_eq!(6, result.point_count());
    assert_eq!(6, result.window_iter().len());
}

#[test]
fn simplify_2() {
    let line = vec![
        [77f32, 613.],
        [689., 650.],
        [710., 467.],
        [220., 200.],
        [120., 300.],
        [100., 100.],
        [77., 613.],
    ];
    let line: Vec<Vec2> = line.into_iter().map(|v| v.into()).collect();
    assert_eq!(6, line.simplify_rdp(1.0).window_iter().len());
}

#[test]
fn simplify_3() {
    let line: Vec<Vec3A> = vec![
        [77f32, 613., 0.].into(),
        [689., 650., 0.].into(),
        [710., 467., 0.].into(),
        [220., 200., 0.].into(),
        [120., 300., 0.].into(),
        [100., 100., 0.].into(),
        [77., 613., 0.].into(),
    ];
    assert_eq!(7, line.simplify_rdp(1.0).len());
    assert_eq!(5, line.simplify_vw(1).len());
}

#[test]
fn simplify_4() {
    let line: Vec<Vec2> = vec![
        (0.0, 3.0).into(),
        (1.0, 2.0).into(),
        (4.0, 1.0).into(),
        (0.0, 0.0).into(),
    ];
    let indices: Vec<usize> = vec![0, 1, 2, 3];
    assert_eq!(
        2,
        indexed_simplify_rdp_2d(&line, &indices, 1.0)
            .windows(2)
            .len()
    );
    assert_eq!(2, line.simplify_rdp(1.0).window_iter().len());
    assert_eq!(4, line.simplify_vw(1).len());
}

#[test]
fn simplify_5() {
    let line: Vec<Vec3> = vec![
        (0.0, 3.0, 0.0).into(),
        (1.0, 2.0, 0.0).into(),
        (4.0, 1.0, 0.0).into(),
        (0.0, 0.0, 0.0).into(),
    ];
    let indices: Vec<usize> = vec![0, 1, 2, 3];
    assert_eq!(
        2,
        indexed_simplify_rdp_3d(&line, &indices, 1.0)
            .windows(2)
            .len()
    );
    assert_eq!(3, line.simplify_rdp(1.0).len());
    assert_eq!(2, line.simplify_vw(1).len());
}

#[test]
fn a_test() -> Result<(), LinestringError> {
    let _l: Vec<[f32; 2]> = vec![
        [651.3134, 410.21536],
        [335.7384, 544.54614],
        [154.29922, 363.10654],
        [425.06284, 255.50153],
        [651.1434, 387.16595],
        [250.0, 300.0],
    ];

    let l: Vec<Vec2> = _l.into_iter().map(|v| v.into()).collect();
    let result = l.is_self_intersecting()?;

    assert!(result);
    Ok(())
}

#[cfg(all(feature = "cgmath", feature = "glam"))]
#[test]
fn another_test() -> Result<(), linestring::LinestringError> {
    let l: Vec<vector_traits::glam::Vec2> = [[0.0, 10.], [5.0, 51.0], [15.0, 51.0]]
        .into_iter()
        .map(|v| v.into())
        .collect();
    let glam_vec2 = l.is_self_intersecting()?;
    let l: Vec<vector_traits::glam::DVec2> = [[0.0, 10.], [5.0, 51.0], [15.0, 51.0]]
        .into_iter()
        .map(|v| v.into())
        .collect();
    let glam_dvec2 = l.is_self_intersecting()?;
    let l: Vec<vector_traits::cgmath::Vector2<f32>> = [[0.0, 10.], [5.0, 51.0], [15.0, 51.0]]
        .into_iter()
        .map(|v| v.into())
        .collect();
    let cgmath_vector2_f32 = l.is_self_intersecting()?;
    let l: Vec<vector_traits::cgmath::Vector2<f64>> = [[0.0, 10.], [5.0, 51.0], [15.0, 51.0]]
        .into_iter()
        .map(|v| v.into())
        .collect();
    let cgmath_vector_f64 = l.is_self_intersecting()?;
    assert_eq!(glam_vec2, glam_dvec2);
    assert_eq!(cgmath_vector2_f32, cgmath_vector_f64);
    assert_eq!(glam_vec2, cgmath_vector_f64);
    assert_eq!(glam_vec2, false);
    Ok(())
}

#[test]
fn simplify_vw_1() {
    let line = vec![
        [77f32, 613.],
        [689., 650.],
        [710., 467.],
        [220., 200.],
        [120., 300.],
        [100., 100.],
        [77., 613.],
    ];
    let line: Vec<Vec2> = line.into_iter().map(|v| v.into()).collect();
    assert_eq!(line.len() - 4, line.simplify_vw(4).len());
}

#[test]
fn simplify_vw_2() {
    let line = vec![
        [0f32, 0.],
        [100., 0.],
        [99., 99.],
        [100., 100.],
        [101., 101.],
        [0., 100.],
        [0f32, 0.],
    ];
    let line: Vec<Vec2> = line.into_iter().map(|v| v.into()).collect();
    let l = line.simplify_vw(2);
    assert_eq!(line.len() - 2, l.len());
    assert!(line.is_connected())
}

#[test]
fn simplify_vw_3() {
    let line = vec![
        [0f32, 0.],
        [100., 0.],
        [100., 100.],
        [0., 100.],
        [10., 11.],
        [1., 12.],
        [3., 1.],
        [0f32, 0.],
    ];
    let line: Vec<Vec2> = line.into_iter().map(|v| v.into()).collect();
    let l = line.simplify_vw(5);
    assert_eq!(line.len() - 5, l.len());
    assert!(line.is_connected())
}

#[test]
fn simplify_vw_4() {
    let mut line: Vec<[f32; 2]> = Vec::with_capacity(360);
    for i in (0..360).step_by(20) {
        let i = i as f32;
        line.push([i.to_radians().cos() * 100f32, i.to_radians().sin() * 100f32])
    }

    let line: Vec<Vec2> = line.into_iter().map(|v| v.into()).collect();
    let is_connected = line.is_connected();
    let l = line.simplify_vw(6);
    assert_eq!(line.len() - 6, l.len());
    assert_eq!(line.is_connected(), is_connected)
}

#[test]
fn simplify_6() {
    let line: Vec<Vec2> = vec![[0f32, 0.].into(), [100., 0.].into()];

    let l = line.simplify_vw(5);
    assert_eq!(2, l.len());
    let l = line.simplify_rdp(5.0);
    assert_eq!(2, l.len());
}

#[test]
fn voronoi_parabolic_arc_1() {
    use vector_traits::glam;
    /*
    point1:Point { x: 200, y: 200 },
    segment:Line { start: Point { x: 100, y: 100 }, end: Point { x: 300, y: 100 } },
    max_dist:0.800000037997961,
    discretization:[[100.0, 200.0], [300.0, 200.0]]
    discretize: result:[[100.0, 200.0], [125.0, 178.125], [150.0, 162.5], [175.0, 153.125], [200.0, 150.0], [225.0, 153.125], [250.0, 162.5], [275.0, 178.125], [300.0, 200.0]]
    */
    let cell_point: glam::Vec2 = [200.0, 200.0].into();
    let segment: linestring_2d::Line2<glam::Vec2> = [[100.0, 100.0], [300.0, 100.0]].into();
    let max_dist: f32 = 0.800000037997961;
    let start_point: glam::Vec2 = [100.0, 200.0].into();
    let end_point: glam::Vec2 = [300.0, 200.0].into();

    let vpa = linestring_2d::VoronoiParabolicArc::new(segment, cell_point, start_point, end_point);
    let result = vpa.discretize_2d(max_dist);
    let epsilon = <Vec2 as HasXY>::Scalar::default_epsilon();
    let ulps = <Vec2 as HasXY>::Scalar::default_max_ulps();
    assert!(result[0].is_ulps_eq([100.0, 200.0].into(), epsilon, ulps));
    assert!(result[1].is_ulps_eq([125.0, 178.125].into(), epsilon, ulps));
    assert!(result[2].is_ulps_eq([150.0, 162.5].into(), epsilon, ulps));
    assert!(result[3].is_ulps_eq([175.0, 153.125].into(), epsilon, ulps));
    assert!(result[4].is_ulps_eq([200.0, 150.0].into(), epsilon, ulps));
    assert!(result[5].is_ulps_eq([225.0, 153.125].into(), epsilon, ulps));
    assert!(result[6].is_ulps_eq([250.0, 162.5].into(), epsilon, ulps));
    assert!(result[7].is_ulps_eq([275.0, 178.125].into(), epsilon, ulps));
    assert!(result[8].is_ulps_eq([300.0, 200.0].into(), epsilon, ulps));
}

#[test]
fn transform_test_1() -> Result<(), linestring::LinestringError> {
    let mut aabb_source = linestring_2d::Aabb2::default();
    let mut aabb_dest = linestring_2d::Aabb2::default();

    // source is (0,0)-(1,1)
    aabb_source.update_with_point(Vec2::new(0., 0.));
    aabb_source.update_with_point(Vec2::new(1., 1.));

    // dest is (1,1)-(2,2)
    aabb_dest.update_with_point(Vec2::new(1., 1.));
    aabb_dest.update_with_point(Vec2::new(2., 2.));

    let transform = linestring_2d::SimpleAffine::new(&aabb_source, &aabb_dest)?;
    assert_eq!(
        transform.transform_ab(Vec2::new(0., 0.))?,
        Vec2::new(1., 1.)
    );
    assert_eq!(
        transform.transform_ab(Vec2::new(1., 1.))?,
        Vec2::new(2., 2.)
    );
    assert_eq!(
        transform.transform_ab(Vec2::new(0., 1.))?,
        Vec2::new(1., 2.)
    );
    assert_eq!(
        transform.transform_ab(Vec2::new(1., 0.))?,
        Vec2::new(2., 1.)
    );

    Ok(())
}

#[test]
fn transform_test_2() -> Result<(), linestring::LinestringError> {
    let mut aabb_source = linestring_2d::Aabb2::default();
    let mut aabb_dest = linestring_2d::Aabb2::default();

    // source is (-100,-100)-(100,100)
    aabb_source.update_with_point(Vec2::new(-100., -100.));
    aabb_source.update_with_point(Vec2::new(100., 100.));

    // dest is (0,0)-(800,800.)
    aabb_dest.update_with_point(Vec2::new(0., 0.));
    aabb_dest.update_with_point(Vec2::new(800., 800.));

    let transform = linestring_2d::SimpleAffine::new(&aabb_source, &aabb_dest)?;
    //println!("Affine:{:?}", transform);

    assert_eq!(
        transform.transform_ab(Vec2::new(-100., -100.))?,
        Vec2::new(0., 0.)
    );
    assert_eq!(
        transform.transform_ba(Vec2::new(0., 0.))?,
        Vec2::new(-100., -100.)
    );
    assert_eq!(
        transform.transform_ab(Vec2::new(100., 100.))?,
        Vec2::new(800., 800.)
    );
    assert_eq!(
        transform.transform_ba(Vec2::new(800., 800.))?,
        Vec2::new(100., 100.)
    );
    Ok(())
}

#[test]
fn convex_hull_0() -> Result<(), linestring::LinestringError> {
    use linestring::linestring_2d::convex_hull;
    let line: Vec<Vec2> = vec![
        [0f32, 0.].into(),
        [100., 0.].into(),
        [100., 100.].into(),
        [0., 100.].into(),
        [1., 1220.].into(),
        [50., 2.].into(),
        [0f32, 0.].into(),
    ];

    let gw = convex_hull::gift_wrap(&line)?;
    let indices: Vec<usize> = (0..line.len()).collect();
    let pch: Vec<Vec2> = convex_hull::convex_hull_par(&line, &indices, 2)?
        .into_iter()
        .map(|i| line[i])
        .collect();
    let pch2 = line.convex_hull_par(2)?;
    assert_eq!(gw, pch);
    assert_eq!(pch, pch2);
    for v in &line {
        // see if all points of `line` are contained within `pch`
        assert!(pch.contains_point_inclusive(*v));
    }
    Ok(())
}

#[test]
fn convex_hull_1() -> Result<(), linestring::LinestringError> {
    use linestring::linestring_2d::convex_hull;
    let line: Vec<Vec2> = vec![
        [0f32, 0.].into(),
        [100., 0.].into(),
        [100., 100.].into(),
        [0., 100.].into(),
        [1., 1220.].into(),
        [50., 2.].into(),
        [0f32, 0.].into(),
    ];

    let gw = convex_hull::gift_wrap(&line)?;
    let gs = convex_hull::graham_scan(&line)?;
    let indices: Vec<usize> = (0..line.len()).collect();
    let igs: Vec<Vec2> = convex_hull::indexed_graham_scan(&line, &indices)?
        .into_iter()
        .map(|i| line[i])
        .collect();
    let igw: Vec<Vec2> = convex_hull::indexed_gift_wrap(&line, &indices)?
        .into_iter()
        .map(|i| line[i])
        .collect();
    let pch: Vec<Vec2> = convex_hull::convex_hull_par(&line, &indices, 2)?
        .into_iter()
        .map(|i| line[i])
        .collect();

    if gw != gs {
        println!("input:{:?}", line);
        println!("gw:{:?}", gw);
        println!("gs:{:?}", gs);
        panic!("test failed")
    }
    assert!(convex_hull::contains_convex_hull(&gw, &gs,));
    assert!(convex_hull::contains_convex_hull(&gs, &gw,));
    assert_eq!(gw, igs);
    assert_eq!(gw, gs);
    assert_eq!(gw, igw);
    assert_eq!(gw, pch);

    Ok(())
}

#[test]
fn convex_hull_2() -> Result<(), LinestringError> {
    use linestring_2d::convex_hull;
    use rand::{rngs::StdRng, Rng, SeedableRng};

    let mut rng: StdRng = SeedableRng::from_seed([42; 32]);
    let mut points = Vec::<Vec2>::new();
    for _i in 0..3023 {
        let p: [f32; 2] = [rng.gen_range(0.0..4096.0), rng.gen_range(0.0..4096.0)];
        points.push(p.into());
    }

    let gs = convex_hull::graham_scan(&points)?;
    let indices: Vec<usize> = (0..points.len()).collect();
    let igs = convex_hull::indexed_graham_scan(&points, &indices)?;
    let igw = convex_hull::indexed_gift_wrap(&points, &indices)?;
    let center = Vec2::new(2000.0, 2000.0);

    for l in gs.window_iter() {
        assert!(convex_hull::is_point_left(l.start, l.end, center));
    }
    assert_eq!(igs.len(), gs.len());
    assert_eq!(igs, igw);
    Ok(())
}

#[test]
fn convex_hull_3() -> Result<(), LinestringError> {
    use linestring_2d::convex_hull;
    use rand::{rngs::StdRng, Rng, SeedableRng};

    let mut rng: StdRng = SeedableRng::from_seed([42; 32]);
    let mut points_a = Vec::<Vec2>::new();
    for _i in 0..1023 {
        let p: [f32; 2] = [rng.gen_range(0.0..4096.0), rng.gen_range(0.0..4096.0)];
        points_a.push(p.into());
    }

    let a = convex_hull::graham_scan(&points_a)?;

    let mut points_b = Vec::<Vec2>::new();
    for _i in 0..1023 {
        let p: [f32; 2] = [rng.gen_range(100.0..3000.0), rng.gen_range(100.0..3000.0)];
        points_b.push(p.into());
    }

    let b = convex_hull::graham_scan(&points_b)?;
    let indices: Vec<usize> = (0..points_b.len()).collect();
    let igs = linestring_2d::convex_hull::indexed_graham_scan(&points_b, &indices)?;

    assert_eq!(b.len(), igs.len());
    assert!(convex_hull::contains_convex_hull(&a, &b));
    assert!(convex_hull::contains_convex_hull_par(&a, &b));
    assert!(!convex_hull::contains_convex_hull(&b, &a));
    assert!(!convex_hull::contains_convex_hull_par(&b, &a));

    #[allow(deprecated)]
    let b = convex_hull::graham_scan_wo_atan2(&points_b)?;

    assert!(convex_hull::contains_convex_hull(&a, &b));
    assert!(convex_hull::contains_convex_hull_par(&a, &b));
    assert!(!convex_hull::contains_convex_hull(&b, &a));
    assert!(!convex_hull::contains_convex_hull_par(&b, &a));
    Ok(())
}

#[test]
fn convex_hull_4() -> Result<(), linestring::LinestringError> {
    use linestring_2d::convex_hull;

    /*let l = LineString2::<Vec2>::with_vec(vec![
        (5.027466, 1.39643).into(),
        (0.572535, 3.40357).into(),
        (1.796431, 0.172535).into(),
        (3.80357, 4.627465).into(),
    ]);
    */
    let vertices: Vec<Vec2> = vec![
        (0.0, 0.0).into(),
        (0.0, 0.0).into(),
        (0.0, 0.0).into(),
        (0.0, 0.0).into(),
        (0.0, 0.0).into(),
        (0.0, 0.0).into(),
        (0.0, 0.0).into(),
        (5.027466, 1.39643).into(),
        (0.572535, 3.40357).into(),
        (1.796431, 0.172535).into(),
        (3.80357, 4.627465).into(),
    ];

    let gw = convex_hull::gift_wrap(&vertices[7..])?;
    let gs = convex_hull::graham_scan(&vertices[7..])?;
    let offset_indices: Vec<usize> = (7..vertices.len()).collect();
    let igs: Vec<Vec2> = convex_hull::indexed_graham_scan(&vertices, &offset_indices)?
        .into_iter()
        .map(|i| vertices[i])
        .collect();
    let igw: Vec<Vec2> = convex_hull::indexed_gift_wrap(&vertices, &offset_indices)?
        .into_iter()
        .map(|i| vertices[i])
        .collect();
    let pch: Vec<Vec2> = convex_hull::convex_hull_par(&vertices, &offset_indices, 2)?
        .into_iter()
        .map(|i| vertices[i])
        .collect();
    if gw != gs {
        println!("input:{:?}", &vertices[7..]);
        println!("gift wrapping:{:?}", gw);
        println!("graham's scan:{:?}", gs);
        panic!("test failed")
    }
    assert_eq!(igs, gs);
    assert_eq!(igs, gw);
    assert_eq!(igs, igw);
    assert_eq!(igs, pch);
    Ok(())
}

#[test]
fn convex_hull_5() -> Result<(), linestring::LinestringError> {
    use linestring::linestring_2d::convex_hull;
    let vertices: Vec<Vec2> = vec![
        (0.0, 0.0).into(),
        (2.0, 3.0).into(), // collinear
        (4.0, 3.0).into(), // collinear
        (6.0, 3.0).into(), // collinear
        (1.0, 1.0).into(),
        (7.0, 4.0).into(),
    ];

    let gw = linestring_2d::convex_hull::gift_wrap(&vertices)?;
    let gs = convex_hull::graham_scan(&vertices)?;
    let indices: Vec<usize> = (0..vertices.len()).collect();
    let igs = convex_hull::indexed_graham_scan(&vertices, &indices)?;
    let igw = convex_hull::indexed_gift_wrap(&vertices, &indices)?;
    let pch = convex_hull::convex_hull_par(&vertices, &indices, 2)?;
    if gw != gs {
        println!("input:{:?}", vertices);
        println!("gift wrapping:{:?}", gw);
        println!("graham's scan:{:?}", gs);
        panic!("test failed")
    }
    assert_eq!(gs.len(), igs.len());
    assert_eq!(gs.len(), igw.len());
    assert_eq!(igs, igw);
    assert_eq!(igs, pch);

    Ok(())
}

/// Generate a convex hull test in a grid pattern, with lots of co-linear edges
#[test]
fn convex_hull_6() -> Result<(), LinestringError> {
    use linestring_2d::convex_hull;

    // Define the grid parameters
    let grid_size = 100; // Number of subgrids in each dimension

    // Calculate the spacing between grid points
    let grid_spacing = 1.0 / (grid_size as f32);
    let mut pos = vec2(0.0, 0.0);
    let mut delta_x = grid_spacing;

    // Generate points in a grid pattern
    let mut points: Vec<Vec2> = Vec::new();
    for _i in 0..grid_size {
        for _j in 0..grid_size {
            points.push(pos);
            pos.x += delta_x;
        }
        points.push(pos);
        delta_x *= -1.0;
        //pos.x = 0.0;
        pos.y += grid_spacing;
    }

    let ghs = convex_hull::graham_scan(&points)?;
    let gw = convex_hull::gift_wrap(&points)?;
    #[allow(deprecated)]
    let ghsw = convex_hull::graham_scan_wo_atan2(&points)?;
    let indices: Vec<usize> = (0..points.len()).collect();
    let igs = convex_hull::indexed_graham_scan(&points, &indices)?;
    let igw = convex_hull::indexed_gift_wrap(&points, &indices)?;
    let chp = convex_hull::convex_hull_par(&points, &indices, 1000)?;
    //println!("ghs.len():{:?}", ghs.len());
    //println!("gw.len():{:?}", gw.len());
    //println!("ghsw.len():{:?}", ghsw.len());

    for chunks in ghs.windows(3) {
        // test f32
        let cross = convex_hull::cross_2d(chunks[0], chunks[1], chunks[2]);
        assert!(cross >= 0.0);
        // test f64
        let a = dvec2(chunks[0].x as f64, chunks[0].y as f64);
        let b = dvec2(chunks[0].x as f64, chunks[0].y as f64);
        let c = dvec2(chunks[0].x as f64, chunks[0].y as f64);
        let cross = convex_hull::cross_2d(a, b, c);
        assert!(
            cross >= 0.0,
            "a:{:?}, b{:?}, c{:?}, cross:{}",
            a,
            b,
            c,
            cross
        );
    }

    assert_eq!(ghs.len(), gw.len());
    assert_eq!(ghs.len(), igs.len());
    assert_eq!(ghs.len(), ghsw.len());
    assert_eq!(ghs, ghsw);
    assert_eq!(ghs, gw);
    assert_eq!(igs, chp);
    assert_eq!(igs, igw);

    for p in points {
        assert!(convex_hull::contains_point_inclusive(&ghs, p));
        assert!(convex_hull::contains_point_inclusive(&gw, p));
        assert!(convex_hull::contains_point_inclusive(&ghsw, p));
    }
    Ok(())
}

#[test]
fn distance_to_line_squared_01() {
    let a = Vec2::new(0.0, 0.0);
    let b = Vec2::new(1.0, 0.0);
    let p = Vec2::new(-2.0, 0.0);
    assert_eq!(linestring_2d::distance_to_line_squared(a, b, p), 4.0);
    let p = Vec2::new(4.0, 0.0);
    assert_eq!(linestring_2d::distance_to_line_squared(a, b, p), 9.0);
    let p = Vec2::new(0.5, 2.0);
    assert_eq!(linestring_2d::distance_to_line_squared(a, b, p), 4.0);
    let p = Vec2::new(0.5, -2.0);
    assert_eq!(linestring_2d::distance_to_line_squared(a, b, p), 4.0);
}

#[test]
fn self_intersecting_1() {
    use vector_traits::glam::vec2;
    let linestring = vec![
        vec2(0.0, 0.0),
        vec2(1.0, 0.0),
        vec2(2.0, 0.0),
        vec2(3.0, 0.0),
        vec2(4.0, 0.0),
        vec2(5.0, 0.0),
        vec2(10.0, 0.0),
        vec2(1.0, 10.0),
        vec2(0.0, 10.0),
        vec2(1.0, 0.5),
    ];
    assert!(!linestring.is_self_intersecting().unwrap());
    // we must add 10 or more points, or the sweepline algorithm won't be run
    let linestring = vec![
        vec2(0.0, 0.0),
        vec2(1.0, 0.0),
        vec2(2.0, 0.0),
        vec2(3.0, 0.0),
        vec2(4.0, 0.0),
        vec2(5.0, 0.0),
        vec2(10.0, 0.0),
        vec2(1.0, 10.0),
        vec2(0.0, 10.0),
        vec2(1.0, -0.5),
    ];
    let rv = linestring.is_self_intersecting();
    assert!(rv.unwrap());
}

#[test]
fn test_save_to_obj_file() {
    use tempfile::tempdir;

    let vertices: Vec<Vec3> = vec![
        (0.0, 0.0, 0.0).into(),
        (2.0, 3.0, 10.0).into(),
        (4.0, 3.0, 0.0).into(),
        (6.0, 3.0, 0.0).into(),
        (1.0, 1.0, 1.0).into(),
        (7.0, 4.0, 0.0).into(),
    ];
    let lines: Vec<Line3<Vec3>> = vertices.chunk_iter().collect();
    // Create a temporary directory
    let temp_dir = tempdir().expect("Failed to create temporary directory");

    // Generate a temporary file path within the temporary directory
    let temp_file_path = temp_dir.path().join("temp.obj");

    // Call the function with the temporary file path
    let result = save_to_obj_file(&temp_file_path, "object_name", &vec![lines]);

    // Assert that the function returned Ok
    assert!(result.is_ok());
}
