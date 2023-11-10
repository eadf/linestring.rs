use crate::prelude::LineString2;
use vector_traits::glam;
use crate::LinestringError;

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

    assert!(line_iter.next().is_none());
    assert!(edge_iter.next().is_none());
}

#[test]
fn readme_example() -> Result<(), LinestringError>{
    use glam::vec2;
    // the vector type can just as well be glam::Dvec2, cgmath::Vector<f32>, cgmath::Vector<f64>,
    let some_points: Vec<glam::Vec2> = vec![
        vec2(77f32, 613.),
        vec2(689., 650.),
        vec2(710., 467.),
        vec2(220., 200.),
    ];
    let convex_hull:Vec<glam::Vec2> = some_points.convex_hull()?;
    for p in some_points {
        assert!(convex_hull.contains_point_inclusive(p));
    }
    Ok(())
}
