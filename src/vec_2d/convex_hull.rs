use crate::vec_2d;
use std::cmp::Ordering;
use std::marker::PhantomData;

pub struct ConvexHull<T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq> {
    pd: PhantomData<T>,
}

impl<T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq> ConvexHull<T> {
    /// finds the point with lowest x
    fn find_lowest_x(linestring: &[[T; 2]]) -> (usize, [T; 2]) {
        let mut lowest = linestring[0];
        let mut index = 0_usize;
        for p in linestring.iter().enumerate().skip(1) {
            if (p.1[0] < lowest[0]) || ((p.1[0] == lowest[0]) && (p.1[1] < lowest[1])) {
                lowest = *p.1;
                index = p.0;
            }
        }
        (index, lowest)
    }

    /// Returns true if the point c lies to the 'left' of the line a->b
    ///```
    /// # use linestring::vec_2d::convex_hull;
    /// let a=[0.0,0.0];
    /// let b=[0.0,10.0];
    /// let c=[-10.0,5.0];
    /// assert!(convex_hull::ConvexHull::is_point_left(&a,&b,&c));
    /// assert!(!convex_hull::ConvexHull::is_point_left(&a,&c,&b));
    /// assert!(convex_hull::ConvexHull::is_point_left(&c,&a,&b));
    /// assert!(!convex_hull::ConvexHull::is_point_left(&c,&b,&a));
    ///```
    #[inline(always)]
    pub fn is_point_left(a: &[T; 2], b: &[T; 2], c: &[T; 2]) -> bool {
        (b[0] - a[0]) * (c[1] - a[1]) > (b[1] - a[1]) * (c[0] - a[0])
    }

    /// distance between two points squared
    #[inline(always)]
    fn distance_squared(a: &[T; 2], b: &[T; 2]) -> T {
        (a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1])
    }

    /// calculate the cross product of two 2d vectors defined by the points a,b,c as: a->b & a-c
    /// return the z coordinate as a scalar.
    /// The return value will be positive if the point c is 'left' of the vector a->b (ccr)
    /// # use linestring::vec_2d::convex_hull;
    /// let a=cgmath::Point2::new(0.0,0.0);
    /// let b=cgmath::Point2::new(0.0,10.0);
    /// let c=cgmath::Point2::new(-10.0,5.0);
    /// assert!(convex_hull::GrahamScan::cross_2d(&a,&b,&c)>0.0);
    /// assert!(convex_hull::GrahamScan::cross_2d(&a,&c,&b)<0.0);
    /// assert!(convex_hull::GrahamScan::cross_2d(&c,&a,&b)>0.0);
    /// assert!(convex_hull::GrahamScan::cross_2d(&c,&b,&a)<0.0);
    #[inline(always)]
    pub fn cross_2d(a: &[T; 2], b: &[T; 2], c: &[T; 2]) -> T {
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    }

    /// finds the convex hull using Gift wrapping algorithm
    /// https://en.wikipedia.org/wiki/Gift_wrapping_algorithm
    pub fn gift_wrap(linestring: &vec_2d::LineString2<T>) -> vec_2d::LineString2<T> {
        if (linestring.len() <= 3 && linestring.connected)
            || (linestring.len() <= 3 && !linestring.connected)
        {
            //println!("shortcut points {:?} connected:{}", linestring.len(), linestring.connected);
            return linestring.clone().with_connected(true);
        }
        let (starting_point, _) = Self::find_lowest_x(&linestring.points);
        let mut rv = vec_2d::LineString2::<T>::with_capacity(linestring.len()).with_connected(true);

        let mut already_on_hull = yabf::Yabf::with_capacity(linestring.len());
        let mut point_on_hull = starting_point;
        let mut end_point: usize;
        let mut i = 0_usize;
        //println!("points {:?}", linestring.len());
        //println!("starting_point {} {:?}", starting_point, linestring.points[starting_point]);
        loop {
            //println!("pushing {}:{:?}", point_on_hull, linestring.points[point_on_hull]);
            rv.points.push(linestring.points[point_on_hull]);
            if point_on_hull != starting_point {
                // don't flag starting point or we won't know when to stop
                already_on_hull.set_bit(point_on_hull, true);
            }
            end_point = 0;
            for (j, sj) in linestring
                .points
                .iter()
                .enumerate()
                .filter(|x| !already_on_hull.bit(x.0))
            {
                if j == end_point {
                    continue;
                }
                if (end_point == point_on_hull)
                    || !Self::is_point_left(&rv.points[i], &linestring.points[end_point], sj)
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
    /// https://en.wikipedia.org/wiki/Graham_scan
    pub fn graham_scan(input: &vec_2d::LineString2<T>) -> vec_2d::LineString2<T> {
        if (input.len() <= 3 && input.connected) || (input.len() <= 3 && !input.connected) {
            //println!("shortcut points {:?} connected:{}", linestring.len(), linestring.connected);
            return input.clone().with_connected(true);
        }
        let mut input_points = input.points.clone();

        let (_starting_index, starting_point) = Self::find_lowest_x(&input_points);

        let comparator = |b: &[T; 2], c: &[T; 2]| {
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
        //println!("graham");
        /*let fake_center = cgmath::Point2::<T>::new(T::from(-10000.0).unwrap(), starting_point[1]);
        for p in input_points.iter() {
            println!(
                "angle:{:?} cross:{:?} distance:{:?}",
                (starting_point[1] - p[1]).atan2(starting_point[0] - p[0]),
                Self::cross_2d(&fake_center, &starting_point, p),
                Self::distance_squared(&starting_point, &p)
            )
        }
        */
        let mut rv = vec_2d::LineString2::<T>::with_capacity(input.len()).with_connected(true);
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
}
