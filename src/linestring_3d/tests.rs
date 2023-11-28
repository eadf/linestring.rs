use crate::{
    linestring_2d::{Line2, VoronoiParabolicArc},
    linestring_3d::{Aabb3, Approx as Approx3, LineString3, Plane},
    prelude::LineString2,
    LinestringError,
};
use vector_traits::{
    approx::{AbsDiffEq, UlpsEq},
    glam,
    glam::{Vec3, Vec3A},
    Approx,
};

#[test]
fn test_linestring_iter_1() {
    let mut line: Vec<Vec3> = vec![
        [77f32, 613., 0.0].into(),
        [689., 650., 0.0].into(),
        [710., 467., 0.0].into(),
        [220., 200., 0.0].into(),
        [120., 300., 0.0].into(),
        [100., 100., 0.0].into(),
        [77., 613., 0.0].into(),
    ];

    assert_eq!(6, line.window_iter().len());
    assert_eq!(3, line.chunk_iter().len());
    let _ = line.pop();
    assert_eq!(5, line.window_iter().len());
    assert_eq!(3, line.chunk_iter().len());
    let _ = line.pop();
    assert_eq!(4, line.window_iter().len());
    assert_eq!(2, line.chunk_iter().len());
    let _ = line.pop();
    assert_eq!(3, line.window_iter().len());
    assert_eq!(2, line.chunk_iter().len());
    let _ = line.pop();
    assert_eq!(2, line.window_iter().len());
    assert_eq!(1, line.chunk_iter().len());
    let _ = line.pop();
    assert_eq!(1, line.window_iter().len());
    assert_eq!(1, line.chunk_iter().len());
    let _ = line.pop();
    assert_eq!(0, line.window_iter().len());
    assert_eq!(0, line.chunk_iter().len());
}

#[test]
fn test_linestring_iter_2() {
    let line: Vec<Vec3> = vec![
        [77f32, 613., 0.0].into(), //0
        [689., 650., 0.0].into(),  //1
        [710., 467., 0.0].into(),  //2
        [220., 200., 0.0].into(),  //3
        [120., 300., 0.0].into(),  //4
        [100., 100., 0.0].into(),  //5
        [77., 613., 0.0].into(),   //6
    ];
    let mut line_iter = line.window_iter();
    let mut edge_iter = line.chunk_iter();

    assert_eq!(line[0], line_iter.next().unwrap().start);
    assert_eq!(line[0], edge_iter.next().unwrap().start);

    assert_eq!(line[1], line_iter.next().unwrap().start);
    assert_eq!(line[2], edge_iter.next().unwrap().start);

    assert_eq!(line[2], line_iter.next().unwrap().start);
    assert_eq!(line[4], edge_iter.next().unwrap().start);

    assert_eq!(line[3], line_iter.next().unwrap().start);
    assert!(edge_iter.next().is_none());

    assert_eq!(line[4], line_iter.next().unwrap().start);
    assert_eq!(line[5], line_iter.next().unwrap().start);
    assert!(line_iter.next().is_none());
}

#[test]
fn test_linestring_iter_3() {
    let line: Vec<Vec3> = vec![
        [77f32, 613., 0.0].into(), //0
    ];
    let mut line_iter = line.window_iter();
    let mut edge_iter = line.chunk_iter();

    assert!(line_iter.next().is_none());
    assert!(edge_iter.next().is_none());
}

#[test]
fn test_discretize_3d_1() -> Result<(), LinestringError> {
    let l = VoronoiParabolicArc::<glam::Vec2>::new(
        Line2::new((0.0, 0.0).into(), (10.0, 0.0).into()),
        (0.5, 0.5).into(),
        (0.0, 0.0).into(),
        (10.0, 0.0).into(),
    );
    let d = l.discretize_3d(0.1.into());
    assert_eq!(d.len(), 10);
    Ok(())
}

#[test]
fn test_aabb3_1() -> Result<(), LinestringError> {
    let aabb3 =
        Aabb3::<glam::DVec3>::with_points(&[(0.0, 0.0, 0.0).into(), (10.0, 10.0, 10.0).into()]);
    assert!(
        aabb3.center().unwrap().is_ulps_eq(
            (5.0, 5.0, 5.0).into(),
            f64::default_epsilon(),
            f64::default_max_ulps()
        ),
        "{:?}",
        aabb3.center().unwrap()
    );
    assert!(aabb3.contains_point_inclusive((0.0, 0.0, 0.0).into()));
    assert!(aabb3.contains_point_inclusive((10.0, 10.0, 0.0).into()));
    assert!(aabb3.contains_point_inclusive((0.0, 10.0, 0.0).into()));
    assert!(aabb3.contains_point_inclusive((10.0, 0.0, 0.0).into()));
    assert!(aabb3.contains_point_inclusive((0.0, 0.0, 10.0).into()));
    assert!(aabb3.contains_point_inclusive((10.0, 10.0, 10.0).into()));
    assert!(aabb3.contains_point_inclusive((0.0, 10.0, 10.0).into()));
    assert!(aabb3.contains_point_inclusive((10.0, 0.0, 10.0).into()));
    Ok(())
}

#[test]
fn test_line_segment_iterator_1() {
    let points = vec![
        Vec3A::new(0.0, 0.0, 0.0),
        Vec3A::new(1.0, 0.0, 0.0),
        Vec3A::new(2.0, 0.0, 0.0),
        Vec3A::new(3.0, 0.0, 0.0),
    ];

    let distance = 1.5;
    let result: Vec<Vec3A> = points.discretize(distance).collect();

    let expected_result = vec![
        Vec3A::new(0.0, 0.0, 0.0),
        Vec3A::new(1.0, 0.0, 0.0),
        Vec3A::new(2.0, 0.0, 0.0),
        Vec3A::new(3.0, 0.0, 0.0),
    ];

    assert_eq!(result.len(), expected_result.len());
    for (result, expected) in result.iter().zip(expected_result) {
        assert!(
            result.is_abs_diff_eq(expected, 1e-6.into()),
            "{:?}!={:?}",
            result,
            expected
        );
    }
}

#[test]
fn test_line_segment_iterator_2() {
    let points = vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(2.0, 0.0, 0.0),
        Vec3::new(3.0, 0.0, 0.0),
    ];

    let distance = 0.8;
    let result: Vec<Vec3> = points.discretize(distance).collect();

    let expected_result = vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.5, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(1.5, 0.0, 0.0),
        Vec3::new(2.0, 0.0, 0.0),
        Vec3::new(2.5, 0.0, 0.0),
        Vec3::new(3.0, 0.0, 0.0),
    ];
    assert_eq!(result.len(), expected_result.len());
    for (result, expected) in result.iter().zip(expected_result) {
        assert!(
            result.is_abs_diff_eq(expected, 1e-6.into()),
            "{:?}!={:?}",
            result,
            expected
        );
    }
}

#[test]
fn test_line_segment_iterator_3() {
    let points = vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(2.0, 0.0, 0.0),
        Vec3::new(3.0, 0.0, 0.0),
    ];

    let distance = 0.99;
    let result: Vec<Vec3> = points.discretize(distance).collect();

    let expected_result = vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.5, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(1.5, 0.0, 0.0),
        Vec3::new(2.0, 0.0, 0.0),
        Vec3::new(2.5, 0.0, 0.0),
        Vec3::new(3.0, 0.0, 0.0),
    ];
    assert_eq!(result.len(), expected_result.len());
    for (result, expected) in result.iter().zip(expected_result) {
        assert!(
            result.is_abs_diff_eq(expected, 1e-6.into()),
            "{:?}!={:?}",
            result,
            expected
        );
    }
}

#[test]
fn test_line_segment_iterator_4() {
    let points = vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 0.0),
    ];

    let distance = 0.8;
    let iterator = points.discretize(distance);

    let expected_result = vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 0.0),
    ];

    for (result, expected) in iterator.zip(expected_result) {
        assert!(
            result.is_abs_diff_eq(expected, 1e-6.into()),
            "{:?}!={:?}",
            result,
            expected
        );
    }
}

#[test]
fn test_line_segment_iterator_5() {
    let points = vec![Vec3::new(0.0, 0.0, 0.0)];

    let distance = 0.8;
    let iterator = points.discretize(distance);

    let expected_result = vec![Vec3::new(0.0, 0.0, 0.0)];

    for (result, expected) in iterator.zip(expected_result) {
        assert!(
            result.is_abs_diff_eq(expected, 1e-6.into()),
            "{:?}!={:?}",
            result,
            expected
        );
    }
}

#[test]
fn test_line_segment_iterator_6() {
    let points = vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)];

    let distance = 1.0;
    let iterator = points.discretize(distance);

    let expected_result = vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)];

    for (result, expected) in iterator.zip(expected_result) {
        assert!(
            result.is_abs_diff_eq(expected, 1e-6.into()),
            "{:?}!={:?}",
            result,
            expected
        );
    }
}

#[test]
fn test_line_segment_iterator_single_point() {
    // Test when there's only one point in the input
    let points = vec![Vec3::new(0.0, 0.0, 0.0)];
    let distance = 1.5;
    let iterator = points.discretize(distance);

    let expected_result = vec![Vec3::new(0.0, 0.0, 0.0)];

    for (result, expected) in iterator.zip(expected_result) {
        assert!(
            result.is_abs_diff_eq(expected, 1e-6.into()),
            "{:?}=={:?}",
            result,
            expected
        );
    }
}

#[test]
fn test_line_segment_iterator_empty() {
    // Test when the input points vector is empty
    let points: Vec<glam::DVec3> = vec![];
    let distance = 1.5;
    let iterator = points.discretize(distance);

    assert_eq!(iterator.count(), 0);
}

#[test]
fn test_get_plane_xy() {
    let points = vec![
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 1.0, 0.0),
    ];
    let aabb = Aabb3::with_points(&points);
    let plane = Plane::get_plane(aabb).unwrap();
    assert_eq!(plane, Plane::XY);
    let p3d = Vec3::new(10.0, 1.0, 0.0);
    let p2d = plane.point_to_2d(p3d);
    let p3d_2 = plane.point_to_3d(p2d);
    assert!(p3d.is_ulps_eq(p3d_2, f32::default_epsilon(), f32::default_max_ulps()));
    let points_2d = points.copy_to_2d(plane);
    let points_3d = points_2d.copy_to_3d(plane);
    assert!(points_3d.ulps_eq(&points, f32::default_epsilon(), f32::default_max_ulps()));
}

#[test]
fn test_get_plane_xz() {
    use vector_traits::glam::vec3;
    let points = vec![
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 2.0),
        Vec3::new(1.0, 0.0, 1.0),
    ];
    let aabb = Aabb3::with_points(&points);
    let plane = Plane::get_plane(aabb).unwrap();
    assert_eq!(plane, Plane::XZ);
    let p3d = Vec3::new(10.0, 0.0, 1.0);
    let p2d = plane.point_to_2d(p3d);
    let p3d_2 = plane.point_to_3d(p2d);

    assert!(p3d.is_ulps_eq(p3d_2, f32::default_epsilon(), f32::default_max_ulps()));
    let points_2d = points.copy_to_2d(plane);
    let mut points_3d = points_2d.copy_to_3d(plane);
    assert!(points_3d.abs_diff_eq(&points, f32::default_epsilon()));
    points_3d.apply(&|x| x + vec3(1.0, 2.0, 3.0));
    assert_eq!(points_3d[0], vec3(2.0, 2.0, 3.0));
    assert_eq!(points_3d[1], vec3(1.0, 2.0, 5.0));
    assert_eq!(points_3d[2], vec3(2.0, 2.0, 4.0));
}

#[test]
fn test_get_plane_yz() {
    let points = vec![
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 2.0),
        Vec3::new(0.0, 10.0, 1.0),
    ];
    let aabb = Aabb3::with_points(&points);
    let plane = Plane::get_plane(aabb).unwrap();
    assert_eq!(plane, Plane::YZ);
    let p3d = Vec3::new(0.0, 10.0, 1.0);
    let p2d = plane.point_to_2d(p3d);
    let p3d_2 = plane.point_to_3d(p2d);

    assert!(p3d.is_ulps_eq(p3d_2, f32::default_epsilon(), f32::default_max_ulps()));
    let points_2d = points.copy_to_2d(plane);
    let points_3d = points_2d.copy_to_3d(plane);
    assert!(
        points_3d.ulps_eq(&points, f32::default_epsilon(), f32::default_max_ulps()),
        "{:?}!={:?}",
        points_3d,
        points
    );
}
