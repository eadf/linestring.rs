use crate::LinestringError;
use itertools::Itertools;
use vector_traits::glam;

#[test]
fn test_indexed_intersection_1() -> Result<(), LinestringError> {
    use crate::linestring_2d::indexed_intersection::IntersectionTester;
    use glam::vec2;

    let vertices: Vec<glam::Vec2> = vec![
        vec2(651.3134, 410.21536),
        vec2(335.7384, 544.54614),
        vec2(154.29922, 363.10654),
        vec2(425.06284, 255.50153),
        vec2(651.1434, 387.16595),
        vec2(250.0, 300.0),
    ];
    let edges: Vec<(usize, usize)> = (0..vertices.len() as usize).tuples().collect();
    let input_vertices_len = vertices.len();

    let (output_vertices, result_iter) = IntersectionTester::new(vertices)
        .with_ignore_end_point_intersections(true)?
        .with_stop_at_first_intersection(true)?
        .with_edges(edges.iter())?
        .compute()?;

    assert_eq!(output_vertices.len(), input_vertices_len + 1);
    assert_eq!(1, result_iter.len());

    Ok(())
}
