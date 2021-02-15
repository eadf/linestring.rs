#[cfg(feature = "impl-cgmath")]
use linestring::cgmath_impl;

#[cfg(feature = "impl-cgmath")]
fn almost_equal<T: cgmath::BaseFloat>(x1: T, x2: T, y1: T, y2: T) {
    assert!(cgmath_impl::ulps_eq(&x1, &x2));
    assert!(cgmath_impl::ulps_eq(&y1, &y2));
}

/// draws a line pivoting around (x,y) with 'angle' in degrees
/// l1 & l2 are lengths
#[cfg(feature = "impl-cgmath")]
fn pivot(x: f64, y: f64, l1: f64, l2: f64, angle: f64) -> cgmath_impl::Line2<f64> {
    let l = cgmath_impl::Line2 {
        start: cgmath::Point2 {
            x: x + angle.to_radians().cos() * l1,
            y: y + angle.to_radians().sin() * l1,
        },
        end: cgmath::Point2 {
            x: x + (angle + 180.0).to_radians().cos() * l2,
            y: y + (angle + 180.0).to_radians().sin() * l2,
        },
    };
    // nudge the result so that it really pivots at the (x,y) point
    let v = l.end - l.start;
    let v1 = -v * (l1 / (l1 + l2));
    let v2 = v * (l2 / (l1 + l2));

    cgmath_impl::Line2 {
        start: cgmath::Point2 {
            x: x + v1.x,
            y: y + v1.y,
        },
        end: cgmath::Point2 {
            x: x + v2.x,
            y: y + v2.y,
        },
    }
}

#[cfg(feature = "impl-cgmath")]
#[test]
fn linestring2_1() {
    let points: Vec<[f32; 2]> = vec![[0., 0.], [1., 1.], [2., 2.], [3., 3.], [1., 10.]];
    let points_len = points.len();

    let mut linestring: cgmath_impl::LineString2<f32> = points.into_iter().collect();
    assert_eq!(linestring.len(), points_len);

    linestring.connected = false;
    //println!("as_lines={:?}", linestring.as_lines());
    assert_eq!(linestring.as_lines().len(), points_len - 1);

    linestring.connected = true;
    //println!("as_lines={:?}", linestring.as_lines());
    assert_eq!(linestring.as_lines().len(), points_len);
}

#[cfg(feature = "impl-cgmath")]
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

    let mut linestring: cgmath_impl::LineString3<f32> = points.into_iter().collect();
    assert_eq!(linestring.len(), points_len);

    linestring.connected = false;
    //println!("as_lines={:?}", linestring.as_lines());
    assert_eq!(linestring.as_lines().len(), points_len - 1);

    linestring.connected = true;
    //println!("as_lines={:?}", linestring.as_lines());
    assert_eq!(linestring.as_lines().len(), points_len);
}

#[cfg(feature = "impl-cgmath")]
#[test]
fn line2_1() {
    let line = cgmath_impl::Line2::<f64>::from([[10., 0.], [0., 11.]]);
    assert_eq!(line.start, cgmath::Point2::from([10., 0.]));
    assert_eq!(line.end, cgmath::Point2::from([0., 11.]));
}

#[cfg(feature = "impl-cgmath")]
#[test]
fn line3_1() {
    let line = cgmath_impl::Line3::<f64>::from([[10., 0., 9.], [0., 11., 9.]]);
    assert_eq!(line.start, cgmath::Point3::from([10., 0., 9.]));
    assert_eq!(line.end, cgmath::Point3::from([0., 11., 9.]));
}

#[cfg(feature = "impl-cgmath")]
#[test]
fn intersection_1() {
    let l1 = cgmath_impl::Line2::from([200., 200., 300., 300.]);
    let l2 = cgmath_impl::Line2::from([400., 200., 300., 300.]);
    let rv = l1.intersection_point(&l2).unwrap().single();
    almost_equal(rv.x, 300.0, rv.y, 300.0);
}

#[cfg(feature = "impl-cgmath")]
#[test]
fn intersection_2() {
    let l1 = cgmath_impl::Line2::from([200., 200., 300., 400.]);
    let l2 = cgmath_impl::Line2::from([400., 200., 300., 400.]);
    let rv = l1.intersection_point(&l2).unwrap().single();
    almost_equal(rv.x, 300.0, rv.y, 400.0);
}

#[cfg(feature = "impl-cgmath")]
#[test]
fn intersection_3() {
    // line to point detection
    let l1 = cgmath_impl::Line2::from([200., 200., 300., 300.]);
    let l2 = cgmath_impl::Line2::from([250., 250., 250., 250.]);
    let rv = l1.intersection_point(&l2).unwrap().single();
    almost_equal(rv.x, 250.0, rv.y, 250.0);
}

#[cfg(feature = "impl-cgmath")]
#[test]
fn intersection_4() {
    // line to point detection
    let l1 = cgmath_impl::Line2::from([300., 300., 200., 200.]);
    let l2 = cgmath_impl::Line2::from([250., 250., 250., 250.]);
    let rv = l1.intersection_point(&l2).unwrap().single();
    almost_equal(rv.x, 250.0, rv.y, 250.0);
}

#[cfg(feature = "impl-cgmath")]
#[test]
fn intersection_5() {
    // line to point detection
    for r in (0..360).step_by(5) {
        let line = pivot(100.0, 100.0, 50.0, 50.0, r as f64);
        let vector = line.end - line.start;
        let point = cgmath_impl::scale_to_coordinate(&line.start, &vector, 0.45);
        //println!("line:{:?}", line);
        //println!("point:{:?}", point);
        let rv = line
            .intersection_point(&cgmath_impl::Line2::new(point, point))
            .unwrap()
            .single();
        //println!("rv:{:?}", rv);
        //println!();
        almost_equal(rv.x, point.x, rv.y, point.y);
    }
}

#[cfg(feature = "impl-cgmath")]
#[test]
fn intersection_6() {
    let line1 = cgmath_impl::Line2::<f64>::new(
        cgmath::Point2 { x: 100.0, y: 150.0 },
        cgmath::Point2 { x: 150.0, y: 100.0 },
    );
    let line2 = cgmath_impl::Line2::<f64>::new(
        cgmath::Point2 { x: 150.0, y: 100.0 },
        cgmath::Point2 { x: 160.0, y: 150.0 },
    );
    let _rv = line1.intersection_point(&line2);
}

#[cfg(feature = "impl-cgmath")]
#[test]
fn intersection_7() {
    // README.md example
    let line1 = cgmath_impl::Line2::<f64>::new(
        cgmath::Point2 { x: 100.0, y: 150.0 },
        cgmath::Point2 { x: 150.0, y: 100.0 },
    );
    let line2 = cgmath_impl::Line2::<f64>::new(
        cgmath::Point2 { x: 100.0, y: 150.0 },
        cgmath::Point2 { x: 150.0, y: 100.0 },
    );
    let _rv = line1.intersection_point(&line2);
    match _rv {
        Some(cgmath_impl::Intersection::Intersection(_a)) => panic!("expected an overlap"),
        Some(cgmath_impl::Intersection::OverLap(_a)) => (), //println!("{:?}", _a),
        None => panic!("expected an overlap"),
    }
    // you can also get a single intersection point from the Intersection enum.
    // Albeit geometrically incorrect, it makes things easy
    if let Some(_rv) = _rv {
        //println!("{:?}", _rv.single());
    }
}

#[cfg(feature = "impl-cgmath")]
#[test]
fn intersection_9() {
    // overlapping lines
    for r in (0..360).step_by(3) {
        let line1 = pivot(200.0, 200.0, 50.0, 1.0, r as f64);
        /*
        println!(
            "line1:{:?} slope:{}",
            line1,
            (line1.start.x - line1.end.x) / (line1.start.y - line1.end.y)
        );*/

        let rv = line1.intersection_point(&line1);
        match rv {
            Some(cgmath_impl::Intersection::Intersection(_a)) => {
                panic!("expected an overlap, got {:?}", _a)
            }
            Some(cgmath_impl::Intersection::OverLap(_a)) => {
                //println!("{:?}", _a);
                assert_eq!(_a.start, line1.start);
                assert_eq!(_a.end, line1.end);
            }
            _ => panic!("expected an overlap, got None"),
        }
    }
}

#[cfg(feature = "impl-cgmath")]
#[test]
fn intersection_10() {
    // overlapping lines
    for r in (0..360).step_by(3) {
        let line1 = pivot(200.0, 200.0, 50.0, 1.0, r as f64);
        let line2 = cgmath_impl::Line2 {
            start: line1.end,
            end: line1.start,
        };

        /*
        println!(
            "line1:{:?} slope:{}",
            line1,
            (line1.start.x - line1.end.x) / (line1.start.y - line1.end.y)
        );
        println!(
            "line2:{:?} slope:{}",
            line2,
            (line2.start.x - line2.end.x) / (line2.start.y - line2.end.y)
        );*/

        let rv = line1.intersection_point(&line2);
        match rv {
            Some(cgmath_impl::Intersection::Intersection(_a)) => {
                panic!("expected an overlap, got {:?}", _a)
            }
            Some(cgmath_impl::Intersection::OverLap(_a)) => {
                //println!("{:?}", _a);
                assert_eq!(_a.start, line2.start);
                assert_eq!(_a.end, line2.end); // todo! shouldnt this be line1.start?
            }
            _ => panic!("expected an overlap, got None"),
        }
    }
}
