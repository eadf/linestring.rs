use crate::nalgebra_2d;
use std::cmp::Ordering;
use std::marker::PhantomData;

pub struct ConvexHull<T: nalgebra::RealField> {
    pd: PhantomData<T>,
}

impl<T: nalgebra::RealField> ConvexHull<T> {
    /// finds the point with lowest x
    fn find_lowest_x(linestring: &[nalgebra::Point2<T>]) -> (usize, nalgebra::Point2<T>) {
        let mut lowest = linestring[0];
        let mut index = 0_usize;
        for p in linestring.iter().enumerate().skip(1) {
            if (p.1.x < lowest.x) || ((p.1.x == lowest.x) && (p.1.y < lowest.y)) {
                lowest = *p.1;
                index = p.0;
            }
        }
        (index, lowest)
    }

    /// Returns true if the point 'c' lies to the 'left' of the line a->b
    ///```
    /// # use linestring::nalgebra_2d::convex_hull;
    /// let a = nalgebra::Point2::new( 0.0,0.0 );
    /// let b = nalgebra::Point2::new( 0.0,10.0 );
    /// let c = nalgebra::Point2::new( -10.0,5.0 );
    /// assert!(convex_hull::ConvexHull::is_point_left(&a, &b, &c));
    /// assert!(!convex_hull::ConvexHull::is_point_left(&a, &c, &b));
    /// assert!(convex_hull::ConvexHull::is_point_left(&c, &a, &b));
    /// assert!(!convex_hull::ConvexHull::is_point_left(&c, &b, &a));
    ///```
    #[inline(always)]
    pub fn is_point_left(
        a: &nalgebra::Point2<T>,
        b: &nalgebra::Point2<T>,
        c: &nalgebra::Point2<T>,
    ) -> bool {
        (b.x - a.x) * (c.y - a.y) > (b.y - a.y) * (c.x - a.x)
    }

    /// Returns true if the point 'c' lies to the 'left' of the line a->b
    /// Returns true even if the point lies on the line a-b
    ///```
    /// # use linestring::nalgebra_2d::convex_hull;
    /// let a = nalgebra::Point2::new( 0.0,0.0 );
    /// let b = nalgebra::Point2::new( 0.0,10.0 );
    /// assert!(convex_hull::ConvexHull::is_point_left_allow_collinear(&a, &b, &b));
    /// assert!(convex_hull::ConvexHull::is_point_left_allow_collinear(&a, &b, &a));
    ///```
    #[inline(always)]
    pub fn is_point_left_allow_collinear(
        a: &nalgebra::Point2<T>,
        b: &nalgebra::Point2<T>,
        c: &nalgebra::Point2<T>,
    ) -> bool {
        (b.x - a.x) * (c.y - a.y) >= (b.y - a.y) * (c.x - a.x)
    }

    /// distance between two points squared
    #[inline(always)]
    fn distance_squared(a: &nalgebra::Point2<T>, b: &nalgebra::Point2<T>) -> T {
        (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)
    }

    /// calculate the cross product of two 2d vectors defined by the points a,b,c as: a->b & a-c
    /// return the z coordinate as a scalar.
    /// The return value will be positive if the point c is 'left' of the vector a->b (ccr)
    ///```
    /// # use linestring::nalgebra_2d::convex_hull;
    /// let a = nalgebra::Point2::new(0.0,0.0);
    /// let b = nalgebra::Point2::new(0.0,10.0);
    /// let c = nalgebra::Point2::new(-10.0,5.0);
    /// assert!(convex_hull::ConvexHull::cross_2d(&a, &b, &c) > 0.0);
    /// assert!(convex_hull::ConvexHull::cross_2d(&a, &c, &b) < 0.0);
    /// assert!(convex_hull::ConvexHull::cross_2d(&c, &a, &b) > 0.0);
    /// assert!(convex_hull::ConvexHull::cross_2d(&c, &b, &a) < 0.0);
    ///```
    #[inline(always)]
    pub fn cross_2d(
        a: &nalgebra::Point2<T>,
        b: &nalgebra::Point2<T>,
        c: &nalgebra::Point2<T>,
    ) -> T {
        (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
    }

    /// finds the convex hull using Gift wrapping algorithm
    /// <https://en.wikipedia.org/wiki/Gift_wrapping_algorithm>
    ///```
    /// # use linestring::nalgebra_2d;
    /// # use linestring::nalgebra_2d::convex_hull;
    /// # use rand::{Rng, SeedableRng};
    /// let mut rng = rand_chacha::ChaCha8Rng::seed_from_u64(38);
    /// let mut points = Vec::<nalgebra::Point2<f32>>::new();
    /// for _i in 0..1023 {
    ///   let p: [f32; 2] = [rng.gen_range(0.0..4096.0), rng.gen_range(0.0..4096.0)];
    ///   points.push(p.into());
    /// }
    ///
    /// let a = nalgebra_2d::LineString2::<f32>::default().with_points(points);
    /// let convex_hull = convex_hull::ConvexHull::gift_wrap(a.points().iter());
    /// let center = nalgebra::Point2::new(2000_f32,2000.0);
    ///
    /// for p in convex_hull.points().iter() {
    ///   for l in convex_hull.as_lines().iter() {
    ///     // all segments should have the center point at the 'left' side
    ///     assert!(convex_hull::ConvexHull::is_point_left(&l.start, &l.end, &center));
    ///     // all points on the hull should be 'left' of every segment in the hull
    ///     assert!(convex_hull::ConvexHull::is_point_left_allow_collinear(&l.start, &l.end, p));
    ///   }
    /// }
    ///```
    pub fn gift_wrap<'a, I>(input: I) -> nalgebra_2d::LineString2<T>
    where
        I: IntoIterator<Item = &'a nalgebra::Point2<T>>,
        T: 'a,
    {
        let input_points = input
            .into_iter()
            .copied()
            .collect::<Vec<nalgebra::Point2<T>>>();

        if input_points.len() <= 3 {
            //println!("shortcut points {:?} connected:{}", linestring.len(), linestring.connected);
            let rv = nalgebra_2d::LineString2::<T>::with_capacity(input_points.len())
                .with_connected(true)
                .with_points(input_points);
            return rv;
        }
        let (starting_point, _) = Self::find_lowest_x(&input_points);
        let mut rv =
            nalgebra_2d::LineString2::<T>::with_capacity(input_points.len()).with_connected(true);

        let mut already_on_hull = yabf::Yabf::with_capacity(input_points.len());
        let mut point_on_hull = starting_point;
        let mut end_point: usize;
        let mut i = 0_usize;
        //println!("points {:?}", linestring.len());
        //println!("starting_point {} {:?}", starting_point, linestring.points[starting_point]);
        loop {
            //println!("pushing {}:{:?}", point_on_hull, linestring.points[point_on_hull]);
            rv.points.push(input_points[point_on_hull]);
            if point_on_hull != starting_point {
                // don't flag starting point or we won't know when to stop
                already_on_hull.set_bit(point_on_hull, true);
            }
            end_point = 0;
            for (j, sj) in input_points
                .iter()
                .enumerate()
                .filter(|x| !already_on_hull.bit(x.0))
            {
                if j == end_point {
                    continue;
                }
                if (end_point == point_on_hull)
                    || !Self::is_point_left(&rv.points[i], &input_points[end_point], sj)
                {
                    end_point = j;
                    //println!("found {}:{:?}", end_point, linestring.points[end_point]);
                }
            }
            i += 1;
            point_on_hull = end_point;

            if end_point == starting_point {
                break;
            }
        }
        rv
    }

    /// finds the convex hull using Grahams scan
    /// <https://en.wikipedia.org/wiki/Graham_scan>
    /// Returns true if the 'a' convex hull entirely contains the 'b' convex hull
    ///```
    /// # use linestring::nalgebra_2d;
    /// # use linestring::nalgebra_2d::convex_hull;
    /// # use rand::{Rng, SeedableRng};
    /// let mut rng = rand_chacha::ChaCha8Rng::seed_from_u64(38);
    /// let mut points = Vec::<nalgebra::Point2<f32>>::new();
    /// for _i in 0..1023 {
    ///   let p: [f32; 2] = [rng.gen_range(0.0..4096.0), rng.gen_range(0.0..4096.0)];
    ///   points.push(p.into());
    /// }
    ///
    /// let a = nalgebra_2d::LineString2::<f32>::default().with_points(points);
    /// let convex_hull = convex_hull::ConvexHull::graham_scan(a.points().iter());
    /// let center = nalgebra::Point2::new(2000_f32,2000.0);
    ///
    /// for p in convex_hull.points().iter() {
    ///   for l in convex_hull.as_lines().iter() {
    ///     // all segments should have the center point at the 'left' side
    ///     assert!(convex_hull::ConvexHull::is_point_left(&l.start, &l.end, &center));
    ///     // all points on the hull should be 'left' of every segment in the hull
    ///     assert!(convex_hull::ConvexHull::is_point_left_allow_collinear(&l.start, &l.end, p));
    ///   }
    /// }
    ///```
    pub fn graham_scan<'a, I>(input: I) -> nalgebra_2d::LineString2<T>
    where
        I: IntoIterator<Item = &'a nalgebra::Point2<T>>,
        T: 'a,
    {
        let mut input_points = input
            .into_iter()
            .copied()
            .collect::<Vec<nalgebra::Point2<T>>>();

        if input_points.len() <= 3 {
            //println!("shortcut points {:?} connected:{}", linestring.len(), linestring.connected);
            let rv = nalgebra_2d::LineString2::<T>::with_capacity(input_points.len())
                .with_connected(true)
                .with_points(input_points);
            return rv;
        }
        //= input.points.clone();

        let (_starting_index, starting_point) = Self::find_lowest_x(&input_points);

        let comparator = |b: &nalgebra::Point2<T>, c: &nalgebra::Point2<T>| {
            let ccr = Self::cross_2d(&starting_point, b, c);
            match T::zero().partial_cmp(&ccr) {
                Some(Ordering::Greater) => Ordering::Greater, //CCR
                Some(Ordering::Less) => Ordering::Less,       //CR
                Some(Ordering::Equal) => {
                    let dist_sb = Self::distance_squared(&starting_point, b);
                    let dist_sc = Self::distance_squared(&starting_point, c);
                    if dist_sb > dist_sc {
                        Ordering::Greater
                    } else {
                        Ordering::Less
                    }
                }
                _ => panic!(),
            }
        };
        // sort the input points so that the edges to the 'right' of starting_point goes first
        input_points.sort_unstable_by(comparator);

        let mut rv =
            nalgebra_2d::LineString2::<T>::with_capacity(input_points.len()).with_connected(true);
        rv.points.push(starting_point);

        for p in input_points.iter() {
            while rv.points.len() > 1 {
                let last = rv.points.len() - 1;
                if T::zero() >= Self::cross_2d(&rv.points[last - 1], &rv.points[last], p) {
                    let _ = rv.points.pop();
                } else {
                    break;
                }
            }
            if rv.points.last().unwrap() != p {
                rv.points.push(*p);
            }
        }
        if (rv.points.len() >= 2) && (rv.points.last().unwrap() == rv.points.first().unwrap()) {
            let _ = rv.points.pop();
        }
        rv
    }

    /// Returns true if the 'a' convex hull entirely contains the 'b' convex hull
    ///```
    /// # use linestring::nalgebra_2d;
    /// # use linestring::nalgebra_2d::convex_hull;
    /// # use rand::{Rng, SeedableRng};
    ///
    /// let mut rng = rand_chacha::ChaCha8Rng::seed_from_u64(38);
    /// let mut points = Vec::<nalgebra::Point2<f32>>::new();
    /// for _i in 0..4023 {
    ///    let p: [f32; 2] = [rng.gen_range(0.0..4096.0), rng.gen_range(0.0..4096.0)];
    ///    points.push(p.into());
    /// }
    ///
    /// let a = nalgebra_2d::LineString2::<f32>::default().with_points(points);
    /// let a = convex_hull::ConvexHull::graham_scan(a.points().iter());
    ///
    /// let mut points = Vec::<nalgebra::Point2<f32>>::new();
    /// for _i in 0..1023 {
    ///    let p: [f32; 2] = [rng.gen_range(1000.0..2000.0), rng.gen_range(1000.0..2000.0)];
    ///    points.push(p.into());
    /// }
    ///
    /// let b = nalgebra_2d::LineString2::<f32>::default().with_points(points);
    /// let b = convex_hull::ConvexHull::graham_scan(b.points().iter());
    ///
    /// assert!(convex_hull::ConvexHull::contains(&a, &b));
    /// assert!(!convex_hull::ConvexHull::contains(&b, &a));
    ///```
    pub fn contains(a: &nalgebra_2d::LineString2<T>, b: &nalgebra_2d::LineString2<T>) -> bool {
        if a.len() <= 1 {
            return false;
        }
        if b.is_empty() {
            return true;
        }
        //println!("a.len() {}, b.len() {}", a.len(), b.len());
        // the intention is that each of these loops should be run in separate threads
        for l in a.as_lines().iter().skip(0).step_by(4) {
            for p in b.points.iter() {
                if !Self::is_point_left(&l.start, &l.end, p) {
                    //println!("The point {:?} is not left of {:?}", p, l);
                    return false;
                }
            }
        }
        for l in a.as_lines().iter().skip(1).step_by(4) {
            for p in b.points.iter() {
                if !Self::is_point_left(&l.start, &l.end, p) {
                    //println!("The point {:?} is not left of {:?}", p, l);
                    return false;
                }
            }
        }
        for l in a.as_lines().iter().skip(2).step_by(4) {
            for p in b.points.iter() {
                if !Self::is_point_left(&l.start, &l.end, p) {
                    println!("The point {:?} is not left of {:?}", p, l);
                    return false;
                }
            }
        }
        for l in a.as_lines().iter().skip(3).step_by(4) {
            for p in b.points.iter() {
                if !Self::is_point_left(&l.start, &l.end, p) {
                    //println!("The point {:?} is not left of {:?}", p, l);
                    return false;
                }
            }
        }
        true
    }
}
