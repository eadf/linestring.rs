use approx;
#[cfg(feature = "impl-vec")]
use linestring::vec_impl;
use num_traits::Float;
use std::fmt;

#[cfg(feature = "impl-vec")]
fn almost_equal<T>(x1: T, x2: T, y1: T, y2: T)
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    assert!(vec_impl::ulps_eq(&x1, &x2));
    assert!(vec_impl::ulps_eq(&y1, &y2));
}

#[cfg(feature = "impl-vec")]
#[test]
fn linestring2_1() {
    let points: Vec<[f32; 2]> = vec![[0., 0.], [1., 1.], [2., 2.], [3., 3.], [1., 10.]];
    let points_len = points.len();

    let mut linestring: vec_impl::LineString2<f32> = points.into_iter().collect();
    assert_eq!(linestring.len(), points_len);

    linestring.connected = false;
    //println!("as_lines={:?}", linestring.as_lines());
    assert_eq!(linestring.as_lines().len(), points_len - 1);

    linestring.connected = true;
    //println!("as_lines={:?}", linestring.as_lines());
    assert_eq!(linestring.as_lines().len(), points_len);
}

#[cfg(feature = "impl-vec")]
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

    let mut linestring: vec_impl::LineString3<f32> = points.into_iter().collect();
    assert_eq!(linestring.len(), points_len);

    linestring.connected = false;
    //println!("as_lines={:?}", linestring.as_lines());
    assert_eq!(linestring.as_lines().len(), points_len - 1);

    linestring.connected = true;
    //println!("as_lines={:?}", linestring.as_lines());
    assert_eq!(linestring.as_lines().len(), points_len);
}

#[cfg(feature = "impl-vec")]
#[test]
fn line2_1() {
    let line = vec_impl::Line2::<f64>::from([[10., 0.], [0., 11.]]);
    assert_eq!(line.start, [10., 0.]);
    assert_eq!(line.end, [0., 11.]);
}

#[cfg(feature = "impl-vec")]
#[test]
fn line3_1() {
    let line = vec_impl::Line3::<f64>::from([[10., 0., 9.], [0., 11., 9.]]);
    assert_eq!(line.start, [10., 0., 9.]);
    assert_eq!(line.end, [0., 11., 9.]);
}

#[cfg(feature = "impl-vec")]
#[test]
fn intersection_1() {
    let l1 = vec_impl::Line2::from([200., 200., 300., 300.]);
    let l2 = vec_impl::Line2::from([400., 200., 300., 300.]);
    let rv = l1.intersection_point(&l2).unwrap().single();
    almost_equal(rv[0], 300.0, rv[1], 300.0);
}
