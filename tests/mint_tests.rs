use approx::ulps_eq;
#[cfg(feature = "impl-mint")]
use linestring::mint_2d;
#[cfg(feature = "impl-mint")]
use linestring::mint_3d;
#[cfg(feature = "impl-mint")]
use num_traits::Float;
#[cfg(feature = "impl-mint")]
use std::fmt;

#[cfg(feature = "impl-mint")]
fn almost_equal<T>(x1: T, x2: T, y1: T, y2: T)
where
    T: Float
        + fmt::Debug
        + approx::AbsDiffEq
        + approx::UlpsEq
        + Sync
        + approx::AbsDiffEq<Epsilon = T>,
{
    assert!(ulps_eq!(&x1, &x2));
    assert!(ulps_eq!(&y1, &y2));
}

#[cfg(feature = "impl-mint")]
#[test]
fn linestring2_1() {
    let points: Vec<[f32; 2]> = vec![[0., 0.], [1., 1.], [2., 2.], [3., 3.], [1., 10.]];
    let points_len = points.len();

    let mut linestring: mint_2d::LineString2<f32> = points.into_iter().collect();
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

    let mut linestring: mint_3d::LineString3<f32> = points.into_iter().collect();
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
    let line = mint_2d::Line2::<f64>::from([[10., 0.], [0., 11.]]);
    assert_eq!(line.start, mint::Point2::from([10., 0.]));
    assert_eq!(line.end, mint::Point2::from([0., 11.]));
}

#[cfg(feature = "impl-mint")]
#[test]
fn line3_1() {
    let line = mint_3d::Line3::<f64>::from([[10., 0., 9.], [0., 11., 9.]]);
    assert_eq!(line.start, mint::Point3::from([10., 0., 9.]));
    assert_eq!(line.end, mint::Point3::from([0., 11., 9.]));
}

#[cfg(feature = "impl-mint")]
#[test]
fn intersection_1() {
    let l1 = mint_2d::Line2::from([200., 200., 300., 300.]);
    let l2 = mint_2d::Line2::from([400., 200., 300., 300.]);
    let rv = l1.intersection_point(&l2).unwrap().single();
    almost_equal(rv.x, 300.0, rv.y, 300.0);
}

#[cfg(feature = "impl-mint")]
#[test]
fn aabb2_1() {
    let aabb = mint_2d::Aabb2::from([200_f32, 200., 400., 400.]);
    let line = mint_2d::Line2::from([201_f32, 250., 300., 300.]);
    assert!(aabb.contains_line_inclusive(&line));

    let aabb = mint_2d::Aabb2::from([200_f32, 200., 400., 400.]);
    let line = mint_2d::Line2::from([199.9999_f32, 250., 300., 300.]);
    assert!(!aabb.contains_line_inclusive(&line));
}

#[cfg(feature = "impl-mint")]
#[test]
fn point_distance_1() {
    let point = mint::Point2::from([100_f32, 200.]);
    let line = mint_2d::Line2::from([100_f32, 100., 300., 100.]);
    let distance = mint_2d::distance_to_line_squared(&line.start, &line.end, &point);
    let correct = 100.0_f32 * 100.0;
    assert!(ulps_eq!(&distance, &correct), "{}!={}", distance, correct);

    let point = mint::Point2::from([150_f32, 200.]);
    let distance = mint_2d::distance_to_line_squared(&line.start, &line.end, &point);
    let correct = 100.0_f32 * 100.0;
    assert!(ulps_eq!(&distance, &correct), "{}!={}", distance, correct);

    let point = mint::Point2::from([54_f32, 0.]);
    let distance = mint_2d::distance_to_line_squared(&line.start, &line.end, &point);
    let correct = 100.0_f32 * 100.0;
    assert!(ulps_eq!(&distance, &correct), "{}!={}", distance, correct);
}

#[cfg(feature = "impl-mint")]
#[test]
fn simplify_1() {
    let linestring: mint_2d::LineString2<f32> = vec![[10f32, 10.], [13.0, 11.0], [20.0, 10.0]]
        .into_iter()
        .collect();
    assert_eq!(1, linestring.simplify(10.0).as_lines().len());
    println!("{:?}", linestring.simplify(0.1));
    assert_eq!(2, linestring.simplify(0.1).as_lines().len());
    let linestring: mint_2d::LineString2<f32> =
        vec![[10f32, 10.], [20.0, 10.0]].into_iter().collect();
    assert_eq!(1, linestring.simplify(0.1).as_lines().len());
    let linestring: mint_2d::LineString2<f32> =
        vec![[10f32, 10.], [15.0, 12.0], [17.0, 13.0], [20.0, 10.0]]
            .into_iter()
            .collect();
    assert_eq!(2, linestring.simplify(1.1).as_lines().len());

    let line = vec![
        [77f32, 613.],
        [689., 650.],
        [710., 467.],
        [220., 200.],
        [120., 300.],
        [100., 100.],
    ];
    let mut line: mint_2d::LineString2<f32> = line.into_iter().collect();
    line.connected = true;
    assert_eq!(6, line.simplify(1.0).as_lines().len());
}

#[cfg(feature = "impl-mint")]
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
    let mut line: mint_2d::LineString2<f32> = line.into_iter().collect();
    line.connected = false;
    assert_eq!(6, line.simplify(1.0).as_lines().len());
}

#[cfg(feature = "impl-mint")]
#[test]
fn triangle_area() {
    for x in -10..10 {
        for y in -10..10 {
            let p1 = mint::Point2 {
                x: x as f32,
                y: 7.0,
            };
            let p2 = mint::Point2 { x: 6_f32, y: 0.0 };
            let p3 = mint::Point2 {
                x: 0_f32,
                y: y as f32,
            };

            let area1 = mint_2d::Line2::triangle_area_squared_times_4(&p1, &p2, &p3);
            //println!("area1 = {}", area1);

            let p1 = mint::Point3 {
                x: x as f32,
                y: 7.0,
                z: 0.0,
            };
            let p2 = mint::Point3 {
                x: 6_f32,
                y: 0.0,
                z: 0.0,
            };
            let p3 = mint::Point3 {
                x: 0_f32,
                y: y as f32,
                z: 0.0,
            };

            let area2 = mint_3d::Line3::triangle_area_squared_times_4(&p1, &p2, &p3);
            //println!("area3 = {}", area2);

            assert!(ulps_eq!(&area1, &area2));
        }
    }
}
