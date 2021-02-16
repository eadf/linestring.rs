#[cfg(feature = "impl-mint")]
use linestring::mint_impl;
use num_traits::Float;
use std::fmt;

#[cfg(feature = "impl-cgmath")]
fn almost_equal<T>(x1: T, x2: T, y1: T, y2: T)
where
    T: Float + fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    assert!(mint_impl::ulps_eq(&x1, &x2));
    assert!(mint_impl::ulps_eq(&y1, &y2));
}

#[cfg(feature = "impl-mint")]
#[test]
fn linestring2_1() {
    let points: Vec<[f32; 2]> = vec![[0., 0.], [1., 1.], [2., 2.], [3., 3.], [1., 10.]];
    let points_len = points.len();

    let mut linestring: mint_impl::LineString2<f32> = points.into_iter().collect();
    assert_eq!(linestring.len(), points_len);

    linestring.connected = false;
    //println!("as_lines={:?}", linestring.as_lines());
    assert_eq!(linestring.as_lines().len(), points_len - 1);

    linestring.connected = true;
    //println!("as_lines={:?}", linestring.as_lines());
    assert_eq!(linestring.as_lines().len(), points_len);
}

#[cfg(feature = "impl-mint")]
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

    let mut linestring: mint_impl::LineString3<f32> = points.into_iter().collect();
    assert_eq!(linestring.len(), points_len);

    linestring.connected = false;
    //println!("as_lines={:?}", linestring.as_lines());
    assert_eq!(linestring.as_lines().len(), points_len - 1);

    linestring.connected = true;
    //println!("as_lines={:?}", linestring.as_lines());
    assert_eq!(linestring.as_lines().len(), points_len);
}

#[cfg(feature = "impl-mint")]
#[test]
fn line2_1() {
    let line = mint_impl::Line2::<f64>::from([[10., 0.], [0., 11.]]);
    assert_eq!(line.start, mint::Point2::from([10., 0.]));
    assert_eq!(line.end, mint::Point2::from([0., 11.]));
}

#[cfg(feature = "impl-mint")]
#[test]
fn line3_1() {
    let line = mint_impl::Line3::<f64>::from([[10., 0., 9.], [0., 11., 9.]]);
    assert_eq!(line.start, mint::Point3::from([10., 0., 9.]));
    assert_eq!(line.end, mint::Point3::from([0., 11., 9.]));
}

#[cfg(feature = "impl-mint")]
#[test]
fn intersection_1() {
    let l1 = mint_impl::Line2::from([200., 200., 300., 300.]);
    let l2 = mint_impl::Line2::from([400., 200., 300., 300.]);
    let rv = l1.intersection_point(&l2).unwrap().single();
    almost_equal(rv.x, 300.0, rv.y, 300.0);
}

#[cfg(feature = "impl-mint")]
#[test]
fn aabb2_1() {
    let aabb = mint_impl::Aabb2::from([200_f32, 200., 400., 400.]);
    let line = mint_impl::Line2::from([201_f32, 250., 300., 300.]);
    assert!(aabb.contains_line(&line));

    let aabb = mint_impl::Aabb2::from([200_f32, 200., 400., 400.]);
    let line = mint_impl::Line2::from([199.9999_f32, 250., 300., 300.]);
    assert!(!aabb.contains_line(&line));
}

#[cfg(feature = "impl-mint")]
#[test]
fn point_distance_1() {
    let point = mint::Point2::from([100_f32, 200.]);
    let line = mint_impl::Line2::from([100_f32, 100., 300., 100.]);
    let distance = mint_impl::distance_to_line2_squared(&line.start, &line.end, &point);
    let correct = 100.0_f32*100.0;
    assert!(mint_impl::ulps_eq(&distance, &correct), "{}!={}",distance, correct);

    let point = mint::Point2::from([150_f32, 200.]);
    let distance = mint_impl::distance_to_line2_squared(&line.start, &line.end, &point);
    let correct = 100.0_f32*100.0;
    assert!(mint_impl::ulps_eq(&distance, &correct), "{}!={}",distance, correct);

    let point = mint::Point2::from([54_f32, 0.]);
    let distance = mint_impl::distance_to_line2_squared(&line.start, &line.end, &point);
    let correct = 100.0_f32*100.0;
    assert!(mint_impl::ulps_eq(&distance, &correct), "{}!={}",distance, correct);
}
