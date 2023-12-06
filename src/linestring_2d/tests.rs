use crate::{
    linestring_2d::{Aabb2, Approx as Approx2, Line2, VoronoiParabolicArc},
    prelude::LineString2,
    LinestringError,
};
use vector_traits::{
    approx::{AbsDiffEq, UlpsEq},
    glam, Approx, Vec2A,
};

#[test]
fn test_linestring_iter_1() {
    use vector_traits::glam::Vec2;
    let mut line: Vec<Vec2> = vec![
        [77f32, 613.].into(),
        [689., 650.].into(),
        [710., 467.].into(),
        [220., 200.].into(),
        [120., 300.].into(),
        [100., 100.].into(),
        [77., 613.].into(),
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
    use vector_traits::glam::Vec2;
    let line: Vec<Vec2> = vec![
        [77f32, 613.].into(), //0
        [689., 650.].into(),  //1
        [710., 467.].into(),  //2
        [220., 200.].into(),  //3
        [120., 300.].into(),  //4
        [100., 100.].into(),  //5
        [77., 613.].into(),   //6
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
    use vector_traits::glam::Vec2;
    let line: Vec<Vec2> = vec![
        [77f32, 613.].into(), //0
    ];
    let mut line_iter = line.window_iter();
    let mut edge_iter = line.chunk_iter();
    assert!(line_iter.is_empty());
    assert!(edge_iter.is_empty());

    assert_eq!(edge_iter.remainder().len(), 1);

    assert!(line_iter.next().is_none());
    assert!(edge_iter.next().is_none());
}

#[test]
fn readme_example() -> Result<(), LinestringError> {
    use glam::vec2;
    // the vector type can just as well be glam::Dvec2, cgmath::Vector<f32>, cgmath::Vector<f64>,
    let some_points: Vec<glam::Vec2> = vec![
        vec2(77f32, 613.),
        vec2(689., 650.),
        vec2(710., 467.),
        vec2(220., 200.),
    ];
    let convex_hull: Vec<glam::Vec2> = some_points.convex_hull()?;
    for p in some_points {
        assert!(convex_hull.contains_point_inclusive(p));
    }
    assert!(convex_hull.contains_point_exclusive(vec2(300.0, 500.0)));
    Ok(())
}

#[test]
fn test_discretize_2d_1() -> Result<(), LinestringError> {
    let l = VoronoiParabolicArc::<glam::Vec2>::new(
        Line2::new((0.0, 0.0).into(), (10.0, 0.0).into()),
        (0.5, 0.5).into(),
        (0.0, 0.0).into(),
        (10.0, 0.0).into(),
    );
    let d = l.discretize_2d(0.1);
    assert_eq!(d.len(), 10);
    Ok(())
}

#[test]
fn test_aabb2_1() -> Result<(), LinestringError> {
    let aabb2 = Aabb2::<glam::DVec2>::with_points(&[(0.0, 0.0).into(), (10.0, 10.0).into()]);
    assert!(aabb2.center().unwrap().is_ulps_eq(
        (5.0, 5.0).into(),
        f64::default_epsilon(),
        f64::default_max_ulps()
    ));
    assert!(aabb2.contains_point_inclusive((0.0, 0.0).into()));
    assert!(aabb2.contains_point_inclusive((10.0, 10.0).into()));
    assert!(aabb2.contains_point_inclusive((0.0, 10.0).into()));
    assert!(aabb2.contains_point_inclusive((10.0, 0.0).into()));
    assert!(aabb2.contains_aabb_inclusive(&aabb2));
    for l in aabb2.convex_hull().unwrap().chunk_iter() {
        assert!(aabb2.contains_line_inclusive(&l));
    }
    let (low, high, delta) = aabb2.extents().unwrap();
    assert_eq!(low, glam::dvec2(0.0, 0.0));
    assert_eq!(high, glam::dvec2(10.0, 10.0));
    assert_eq!(delta.x, 10.0);
    assert_eq!(delta.y, 10.0);

    let aabb2 = Aabb2::<glam::DVec2>::default();
    assert!(aabb2.extents().is_none());
    assert!(aabb2.min_max().is_none());

    let mut aabb2 = Aabb2::<glam::DVec2>::new(glam::dvec2(0.0, 0.0));
    assert!(aabb2.extents().is_some());
    assert!(aabb2.min_max().is_some());

    let aabb = Aabb2::<glam::DVec2>::with_points(&[(1.0, 1.0).into(), (11.0, 11.0).into()]);
    aabb2.update_with_aabb(aabb);
    assert!(aabb2.extents().is_some());
    assert!(aabb2.min_max().is_some());
    assert!(aabb2.contains_aabb_inclusive(&aabb));
    Ok(())
}

#[test]
fn test_line_segment_iterator_1() {
    let points = vec![
        Vec2A::new(0.0, 0.0),
        Vec2A::new(1.0, 0.0),
        Vec2A::new(2.0, 0.0),
        Vec2A::new(3.0, 0.0),
    ];

    let distance = 1.5;
    let result: Vec<Vec2A> = points.discretize(distance).collect();

    let expected_result = vec![
        Vec2A::new(0.0, 0.0),
        Vec2A::new(1.0, 0.0),
        Vec2A::new(2.0, 0.0),
        Vec2A::new(3.0, 0.0),
    ];

    assert_eq!(result.len(), expected_result.len());
    for (result, expected) in result.iter().zip(expected_result) {
        assert!(
            result.is_abs_diff_eq(expected, 1e-6),
            "{:?}!={:?}",
            result,
            expected
        );
    }
}

#[test]
fn test_line_segment_iterator_2() {
    let points = vec![
        glam::Vec2::new(0.0, 0.0),
        glam::Vec2::new(1.0, 0.0),
        glam::Vec2::new(2.0, 0.0),
        glam::Vec2::new(3.0, 0.0),
    ];

    let distance = 0.8;
    let result: Vec<glam::Vec2> = points.discretize(distance).collect();

    let expected_result = vec![
        glam::Vec2::new(0.0, 0.0),
        glam::Vec2::new(0.5, 0.0),
        glam::Vec2::new(1.0, 0.0),
        glam::Vec2::new(1.5, 0.0),
        glam::Vec2::new(2.0, 0.0),
        glam::Vec2::new(2.5, 0.0),
        glam::Vec2::new(3.0, 0.0),
    ];
    assert_eq!(result.len(), expected_result.len());
    assert!(result.abs_diff_eq(&expected_result, f32::default_epsilon()));
    assert!(result.ulps_eq(
        &expected_result,
        f32::default_epsilon(),
        f32::default_max_ulps()
    ));
    for (result, expected) in result.iter().zip(expected_result) {
        assert!(
            result.is_abs_diff_eq(expected, 1e-6),
            "{:?}!={:?}",
            result,
            expected
        );
    }
}

#[test]
fn test_line_segment_iterator_3() {
    let points = vec![
        glam::Vec2::new(0.0, 0.0),
        glam::Vec2::new(1.0, 0.0),
        glam::Vec2::new(2.0, 0.0),
        glam::Vec2::new(3.0, 0.0),
    ];

    let distance = 0.99;
    let result: Vec<glam::Vec2> = points.discretize(distance).collect();

    let expected_result = vec![
        glam::Vec2::new(0.0, 0.0),
        glam::Vec2::new(0.5, 0.0),
        glam::Vec2::new(1.0, 0.0),
        glam::Vec2::new(1.5, 0.0),
        glam::Vec2::new(2.0, 0.0),
        glam::Vec2::new(2.5, 0.0),
        glam::Vec2::new(3.0, 0.0),
    ];
    assert_eq!(result.len(), expected_result.len());
    for (result, expected) in result.iter().zip(expected_result) {
        assert!(
            result.is_abs_diff_eq(expected, 1e-6),
            "{:?}!={:?}",
            result,
            expected
        );
    }
}

#[test]
fn test_line_segment_iterator_4() {
    let points = vec![
        glam::Vec2::new(0.0, 0.0),
        glam::Vec2::new(0.0, 0.0),
        glam::Vec2::new(0.0, 0.0),
    ];

    let distance = 0.8;
    let iterator = points.discretize(distance);

    let expected_result = vec![
        glam::Vec2::new(0.0, 0.0),
        glam::Vec2::new(0.0, 0.0),
        glam::Vec2::new(0.0, 0.0),
    ];

    for (result, expected) in iterator.zip(expected_result) {
        assert!(
            result.is_abs_diff_eq(expected, 1e-6),
            "{:?}!={:?}",
            result,
            expected
        );
    }
}

#[test]
fn test_line_segment_iterator_5() {
    let points = vec![glam::Vec2::new(0.0, 0.0)];

    let distance = 0.8;
    let iterator = points.discretize(distance);

    let expected_result = vec![glam::Vec2::new(0.0, 0.0)];

    for (result, expected) in iterator.zip(expected_result) {
        assert!(
            result.is_abs_diff_eq(expected, 1e-6),
            "{:?}!={:?}",
            result,
            expected
        );
    }
}

#[test]
fn test_line_segment_iterator_6() {
    let points = vec![glam::Vec2::new(0.0, 0.0), glam::Vec2::new(0.0, 1.0)];

    let distance = 1.0;
    let iterator = points.discretize(distance);

    let expected_result = vec![glam::Vec2::new(0.0, 0.0), glam::Vec2::new(0.0, 1.0)];

    for (result, expected) in iterator.zip(expected_result) {
        assert!(
            result.is_abs_diff_eq(expected, 1e-6),
            "{:?}!={:?}",
            result,
            expected
        );
    }
}

#[test]
fn test_line_segment_iterator_single_point() {
    // Test when there's only one point in the input
    let points = vec![glam::Vec2::new(0.0, 0.0)];
    let distance = 1.5;
    let iterator = points.discretize(distance);

    let expected_result = vec![glam::Vec2::new(0.0, 0.0)];

    for (result, expected) in iterator.zip(expected_result) {
        assert!(
            result.is_abs_diff_eq(expected, 1e-6),
            "{:?}=={:?}",
            result,
            expected
        );
    }
}

#[test]
fn test_line_segment_iterator_empty() {
    // Test when the input points vector is empty
    let points: Vec<glam::DVec2> = vec![];
    let distance = 1.5;
    let iterator = points.discretize(distance);

    assert_eq!(iterator.count(), 0);
}
