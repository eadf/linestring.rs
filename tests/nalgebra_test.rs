#[cfg(feature = "impl-nalgebra")]
use linestring::nalgebra_2d;
#[cfg(feature = "impl-nalgebra")]
use linestring::nalgebra_3d;
use num_traits::Float;

#[cfg(feature = "impl-nalgebra")]
fn almost_equal<T>(x1: T, x2: T, y1: T, y2: T)
where
    T: nalgebra::RealField + Float,
{
    assert!(nalgebra_2d::ulps_eq(&x1, &x2));
    assert!(nalgebra_2d::ulps_eq(&y1, &y2));
}

#[cfg(feature = "impl-nalgebra")]
#[test]
fn linestring2_1() {
    let points: Vec<[f32; 2]> = vec![[0., 0.], [1., 1.], [2., 2.], [3., 3.], [1., 10.]];
    let points_len = points.len();

    let mut linestring: nalgebra_2d::LineString2<f32> = points.into_iter().collect();
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

    let mut linestring: nalgebra_3d::LineString3<f32> = points.into_iter().collect();
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
    let line = nalgebra_2d::Line2::<f64>::from([[10., 0.], [0., 11.]]);
    assert_eq!(line.start, nalgebra::Point2::from([10., 0.]));
    assert_eq!(line.end, nalgebra::Point2::from([0., 11.]));
}

#[cfg(feature = "impl-nalgebra")]
#[test]
fn line3_1() {
    let line = nalgebra_3d::Line3::<f64>::from([[10., 0., 9.], [0., 11., 9.]]);
    assert_eq!(line.start, nalgebra::Point3::from([10., 0., 9.]));
    assert_eq!(line.end, nalgebra::Point3::from([0., 11., 9.]));
}

#[cfg(feature = "impl-nalgebra")]
#[test]
fn intersection_1() {
    let l1 = nalgebra_2d::Line2::from([200., 200., 300., 300.]);
    let l2 = nalgebra_2d::Line2::from([400., 200., 300., 300.]);
    let rv = l1.intersection_point(&l2).unwrap().single();
    almost_equal(rv.x, 300.0, rv.y, 300.0);
}

#[cfg(feature = "impl-nalgebra")]
#[test]
fn aabb2_1() {
    let aabb = nalgebra_2d::Aabb2::from([200_f32, 200., 400., 400.]);
    let line = nalgebra_2d::Line2::from([201_f32, 250., 300., 300.]);
    assert!(aabb.contains_line(&line));

    let aabb = nalgebra_2d::Aabb2::from([200_f32, 200., 400., 400.]);
    let line = nalgebra_2d::Line2::from([199.9999_f32, 250., 300., 300.]);
    assert!(!aabb.contains_line(&line));
}

#[cfg(feature = "impl-nalgebra")]
#[test]
fn d2_to_3d_to_2d_1() {
    let plane = nalgebra_3d::Plane::XY;
    let ls2_1 = nalgebra_2d::LineString2::with_iter(
        [
            nalgebra::Point2::new(200_f32, 200.),
            nalgebra::Point2::new(400., 400.),
            nalgebra::Point2::new(400., 200.),
        ]
        .iter(),
    );
    let ls3 = ls2_1.copy_to_3d(plane);
    let ls2_2 = ls3.copy_to_2d(plane);
    assert_eq!(ls2_1.points(), ls2_2.points());
}
