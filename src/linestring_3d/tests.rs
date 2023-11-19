use crate::{
    linestring_2d::{Line2, VoronoiParabolicArc},
    linestring_3d::{Aabb3, LineString3},
    LinestringError,
};
use vector_traits::{
    approx::{AbsDiffEq, UlpsEq},
    glam,
    glam::Vec3,
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
