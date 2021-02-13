#[cfg(feature = "impl-nalgebra")]
use linestring::nalgebra_impl;

#[cfg(feature = "impl-nalgebra")]
#[test]
fn linestring2_1() {
    let points: Vec<[f32; 2]> = vec![[0., 0.], [1., 1.], [2., 2.], [3., 3.], [1., 10.]];
    let points_len = points.len();

    let mut linestring: nalgebra_impl::LineString2<f32> = points.into_iter().collect();
    assert_eq!(linestring.len(), points_len);

    linestring.connected = false;
    //println!("as_lines={:?}", linestring.as_lines());
    assert_eq!(linestring.as_lines().len(), points_len - 1);

    linestring.connected = true;
    //println!("as_lines={:?}", linestring.as_lines());
    assert_eq!(linestring.as_lines().len(), points_len);
}

#[cfg(feature = "impl-nalgebra")]
#[test]
fn linestring3_1() {
    let points: Vec<[f32; 3]> = vec![
        [0., 0., 0.],
        [1., 1., 0.],
        [2., 2., 0.],
        [3., 3., 0.],
        [1., 10., 0.],
    ];
    let points_len = points.len();

    let mut linestring: nalgebra_impl::LineString3<f32> = points.into_iter().collect();
    assert_eq!(linestring.len(), points_len);

    linestring.connected = false;
    //println!("as_lines={:?}", linestring.as_lines());
    assert_eq!(linestring.as_lines().len(), points_len - 1);

    linestring.connected = true;
    //println!("as_lines={:?}", linestring.as_lines());
    assert_eq!(linestring.as_lines().len(), points_len);
}

#[cfg(feature = "impl-nalgebra")]
#[test]
fn line2_1() {
    let line = nalgebra_impl::Line2::<f64>::from([[10., 0.], [0., 11.]]);
    assert_eq!(line.start, nalgebra::Point2::from([10., 0.]));
    assert_eq!(line.end, nalgebra::Point2::from([0., 11.]));
}

#[cfg(feature = "impl-nalgebra")]
#[test]
fn line3_1() {
    let line = nalgebra_impl::Line3::<f64>::from([[10., 0., 9.], [0., 11., 9.]]);
    assert_eq!(line.start, nalgebra::Point3::from([10., 0., 9.]));
    assert_eq!(line.end, nalgebra::Point3::from([0., 11., 9.]));
}
