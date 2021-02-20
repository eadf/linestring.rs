#[cfg(feature = "impl-cgmath")]
use linestring::cgmath_2d;
#[cfg(feature = "impl-cgmath")]
use linestring::cgmath_3d;

#[cfg(feature = "impl-cgmath")]
fn almost_equal<T: cgmath::BaseFloat>(x1: T, x2: T, y1: T, y2: T) {
    assert!(cgmath_2d::ulps_eq(&x1, &x2));
    assert!(cgmath_2d::ulps_eq(&y1, &y2));
}

/// draws a line pivoting around (x,y) with 'angle' in degrees
/// l1 & l2 are lengths
#[cfg(feature = "impl-cgmath")]
fn pivot(x: f64, y: f64, l1: f64, l2: f64, angle: f64) -> cgmath_2d::Line2<f64> {
    let l = cgmath_2d::Line2 {
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

    cgmath_2d::Line2 {
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

    let mut linestring: cgmath_2d::LineString2<f32> = points.into_iter().collect();
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
    let points = vec![
        [0_f32, 0., 0.],
        [1., 1., 0.],
        [2., 2., 0.],
        [3., 3., 0.],
        [1., 10., 0.],
    ];
    let points_len = points.len();

    let mut linestring: cgmath_3d::LineString3<f32> = points.into_iter().collect();
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
    let line = cgmath_2d::Line2::<f64>::from([[10., 0.], [0., 11.]]);
    assert_eq!(line.start, cgmath::Point2::from([10., 0.]));
    assert_eq!(line.end, cgmath::Point2::from([0., 11.]));
}

#[cfg(feature = "impl-cgmath")]
#[test]
fn line3_1() {
    let line = cgmath_3d::Line3::<f64>::from([[10., 0., 9.], [0., 11., 9.]]);
    assert_eq!(line.start, cgmath::Point3::from([10., 0., 9.]));
    assert_eq!(line.end, cgmath::Point3::from([0., 11., 9.]));
}

#[cfg(feature = "impl-cgmath")]
#[test]
fn intersection_1() {
    let l1 = cgmath_2d::Line2::from([200., 200., 300., 300.]);
    let l2 = cgmath_2d::Line2::from([400., 200., 300., 300.]);
    let rv = l1.intersection_point(&l2).unwrap().single();
    almost_equal(rv.x, 300.0, rv.y, 300.0);
}

#[cfg(feature = "impl-cgmath")]
#[test]
fn intersection_2() {
    let l1 = cgmath_2d::Line2::from([200., 200., 300., 400.]);
    let l2 = cgmath_2d::Line2::from([400., 200., 300., 400.]);
    let rv = l1.intersection_point(&l2).unwrap().single();
    almost_equal(rv.x, 300.0, rv.y, 400.0);
}

#[cfg(feature = "impl-cgmath")]
#[test]
fn intersection_3() {
    // line to point detection
    let l1 = cgmath_2d::Line2::from([200., 200., 300., 300.]);
    let l2 = cgmath_2d::Line2::from([250., 250., 250., 250.]);
    let rv = l1.intersection_point(&l2).unwrap().single();
    almost_equal(rv.x, 250.0, rv.y, 250.0);
}

#[cfg(feature = "impl-cgmath")]
#[test]
fn intersection_4() {
    // line to point detection
    let l1 = cgmath_2d::Line2::from([300., 300., 200., 200.]);
    let l2 = cgmath_2d::Line2::from([250., 250., 250., 250.]);
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
        let point = cgmath_2d::scale_to_coordinate(&line.start, &vector, 0.45);
        //println!("line:{:?}", line);
        //println!("point:{:?}", point);
        let rv = line
            .intersection_point(&cgmath_2d::Line2::new(point, point))
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
    let line1 = cgmath_2d::Line2::<f64>::new(
        cgmath::Point2 { x: 100.0, y: 150.0 },
        cgmath::Point2 { x: 150.0, y: 100.0 },
    );
    let line2 = cgmath_2d::Line2::<f64>::new(
        cgmath::Point2 { x: 150.0, y: 100.0 },
        cgmath::Point2 { x: 160.0, y: 150.0 },
    );
    let _rv = line1.intersection_point(&line2);
}

#[cfg(feature = "impl-cgmath")]
#[test]
fn intersection_7() {
    // README.md example
    let line1 = cgmath_2d::Line2::<f64>::new(
        cgmath::Point2 { x: 100.0, y: 150.0 },
        cgmath::Point2 { x: 150.0, y: 100.0 },
    );
    let line2 = cgmath_2d::Line2::<f64>::new(
        cgmath::Point2 { x: 100.0, y: 150.0 },
        cgmath::Point2 { x: 150.0, y: 100.0 },
    );
    let _rv = line1.intersection_point(&line2);
    match _rv {
        Some(cgmath_2d::Intersection::Intersection(_a)) => panic!("expected an overlap"),
        Some(cgmath_2d::Intersection::OverLap(_a)) => (), //println!("{:?}", _a),
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
            Some(cgmath_2d::Intersection::Intersection(_a)) => {
                panic!("expected an overlap, got {:?}", _a)
            }
            Some(cgmath_2d::Intersection::OverLap(_a)) => {
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
        let line2 = cgmath_2d::Line2 {
            start: line1.end,
            end: line1.start,
        };

        let rv = line1.intersection_point(&line2);
        match rv {
            Some(cgmath_2d::Intersection::Intersection(_a)) => {
                panic!("expected an overlap, got {:?}", _a)
            }
            Some(cgmath_2d::Intersection::OverLap(_a)) => {
                //println!("{:?}", _a);
                assert_eq!(_a.start, line2.start);
                assert_eq!(_a.end, line2.end); // todo! shouldnt this be line1.start?
            }
            _ => panic!("expected an overlap, got None"),
        }
    }
}

#[cfg(feature = "impl-cgmath")]
#[test]
fn simplify_1() {
    let linestring: cgmath_2d::LineString2<f32> = vec![[10f32, 10.], [13.0, 11.0], [20.0, 10.0]]
        .into_iter()
        .collect();
    assert_eq!(1, linestring.simplify(10.0).as_lines().len());
    println!("{:?}", linestring.simplify(0.1));
    assert_eq!(2, linestring.simplify(0.1).as_lines().len());
    let linestring: cgmath_2d::LineString2<f32> =
        vec![[10f32, 10.], [20.0, 10.0]].into_iter().collect();
    assert_eq!(1, linestring.simplify(0.1).as_lines().len());
    let linestring: cgmath_2d::LineString2<f32> =
        vec![[10f32, 10.], [15.0, 12.0], [17.0, 13.0], [20.0, 10.0]]
            .into_iter()
            .collect();
    //println!("-----Result :{:?}", linestring.simplify(1.1));
    assert_eq!(2, linestring.simplify(1.1).as_lines().len());

    let line = vec![
        [77f32, 613.],
        [689., 650.],
        [710., 467.],
        [220., 200.],
        [120., 300.],
        [100., 100.],
    ];
    let mut line: cgmath_2d::LineString2<f32> = line.into_iter().collect();
    line.connected = true;
    assert_eq!(6, line.simplify(1.0).as_lines().len());
}

#[cfg(feature = "impl-cgmath")]
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
    let mut line: cgmath_2d::LineString2<f32> = line.into_iter().collect();
    line.connected = false;
    assert_eq!(6, line.simplify(1.0).as_lines().len());
}

#[cfg(feature = "impl-cgmath")]
#[test]
fn simplify_3() {
    let line = vec![
        [77f32, 613., 0.],
        [689., 650., 0.],
        [710., 467., 0.],
        [220., 200., 0.],
        [120., 300., 0.],
        [100., 100., 0.],
        [77., 613., 0.],
    ];
    let mut line: cgmath_3d::LineString3<f32> = line.into_iter().collect();
    line.connected = false;
    assert_eq!(6, line.simplify(1.0).as_lines().len());
}

#[cfg(feature = "impl-cgmath")]
#[test]
fn a_test() -> Result<(), linestring::LinestringError> {
    let _l: Vec<[f32; 2]> = vec![
        [651.3134, 410.21536],
        [335.7384, 544.54614],
        [154.29922, 363.10654],
        [425.06284, 255.50153],
        [651.1434, 387.16595],
        [250.0, 300.0],
    ];

    let mut l: cgmath_2d::LineString2<f32> = _l.into_iter().collect();
    l.connected = true;
    let result = l.is_self_intersecting()?;

    assert!(result);
    Ok(())
}

#[cfg(feature = "impl-cgmath")]
#[test]
fn triangle_area() {
    for x in -10..10 {
        for y in -10..10 {
            let p1 = cgmath::Point2 {
                x: x as f32,
                y: 7.0,
            };
            let p2 = cgmath::Point2 { x: 6_f32, y: 0.0 };
            let p3 = cgmath::Point2 {
                x: 0_f32,
                y: y as f32,
            };

            let area1 = cgmath_2d::Line2::triangle_area_squared_times_4(&p1, &p2, &p3);
            //println!("area1 = {}", area1);

            let p1 = cgmath::Point3 {
                x: x as f32,
                y: 7.0,
                z: 0.0,
            };
            let p2 = cgmath::Point3 {
                x: 6_f32,
                y: 0.0,
                z: 0.0,
            };
            let p3 = cgmath::Point3 {
                x: 0_f32,
                y: y as f32,
                z: 0.0,
            };

            let area2 = cgmath_3d::Line3::triangle_area_squared_times_4(&p1, &p2, &p3);
            //println!("area3 = {}", area2);

            assert!(cgmath_2d::ulps_eq(&area1, &area2));
        }
    }
}
