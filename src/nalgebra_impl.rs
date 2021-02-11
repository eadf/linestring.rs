use std::cmp;
use std::fmt;

// Auto generated file, do NOT edit

#[derive(PartialEq, Eq, Copy, Clone, Hash)]
pub struct Line2<T>
where
    T: nalgebra::Scalar + Copy + fmt::Debug + cmp::PartialOrd,
{
    pub start: nalgebra::Point2<T>,
    pub end: nalgebra::Point2<T>,
}

/// A set of linestrings + an aabb
/// Intended to contain shapes with holes, e.g. outlines of letters
#[derive(PartialEq, Eq, Clone, Hash)]
pub struct LineStringSet2<T>
where
    T: nalgebra::Scalar + Copy + fmt::Debug + cmp::PartialOrd,
{
    set: Vec<LineString2<T>>,
    aabb: Aabb2<T>,
}

#[derive(PartialEq, Eq, Clone, Hash)]
pub struct Aabb2<T>
where
    T: nalgebra::Scalar + Copy + fmt::Debug + cmp::PartialOrd,
{
    aabb_min: Option<nalgebra::Point2<T>>,
    aabb_max: Option<nalgebra::Point2<T>>,
}

#[derive(PartialEq, Eq, Clone, Hash)]
pub struct LineString2<T>
where
    T: nalgebra::Scalar + Copy + fmt::Debug + cmp::PartialOrd,
{
    points: Vec<nalgebra::Point2<T>>,
    connected: bool,
}

#[derive(PartialEq, Eq, Copy, Clone, Hash)]
pub struct Line3<T>
where
    T: nalgebra::Scalar + Copy + fmt::Debug + cmp::PartialOrd,
{
    pub start: nalgebra::Point3<T>,
    pub end: nalgebra::Point3<T>,
}

#[derive(PartialEq, Eq, Clone, Hash)]
pub struct LineString3<T>
where
    T: nalgebra::Scalar + Copy + fmt::Debug + cmp::PartialOrd,
{
    points: Vec<nalgebra::Point3<T>>,
    // would name this 'loop' but it's reserved
    connected: bool,
}

/// A set of linestrings + an aabb
/// Intended to contain shapes with holes, e.g. outlines of letters
#[derive(PartialEq, Eq, Clone, Hash)]
pub struct LineStringSet3<T>
where
    T: nalgebra::Scalar + Copy + fmt::Debug + cmp::PartialOrd,
{
    set: Vec<LineString3<T>>,
    aabb: Aabb3<T>,
}

#[derive(PartialEq, Eq, Clone, Hash)]
pub struct Aabb3<T>
where
    T: nalgebra::Scalar + Copy + fmt::Debug + cmp::PartialOrd,
{
    aabb_min: Option<nalgebra::Point3<T>>,
    aabb_max: Option<nalgebra::Point3<T>>,
}

impl<T> LineString2<T>
where
    T: nalgebra::Scalar + Copy + fmt::Debug + cmp::PartialOrd,
{
    pub fn default() -> Self {
        Self {
            points: Vec::<nalgebra::Point2<T>>::new(),
            connected: false,
        }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::<nalgebra::Point2<T>>::with_capacity(capacity),
            connected: false,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    pub fn as_lines(&self) -> Vec<Line2<T>> {
        if self.points.is_empty() {
            return vec![];
        } else if self.points.len() == 1 {
            return vec![Line2 {
                start: *self.points.first().unwrap(),
                end: *self.points.first().unwrap(),
            }];
        }
        let iter1 = self.points.iter().skip(1);
        let iter2 = self.points.iter();
        if self.connected {
            iter1
                .zip(iter2)
                .map(|(a, b)| Line2 { start: *b, end: *a })
                .chain(
                    Some(Line2 {
                        start: *self.points.first().unwrap(),
                        end: *self.points.last().unwrap(),
                    })
                    .into_iter(),
                )
                .collect()
        } else {
            iter1
                .zip(iter2)
                .map(|(a, b)| Line2 { start: *a, end: *b })
                .collect()
        }
    }

    /// Simplify using Ramer–Douglas–Peucker algorithm
    pub fn simplify(&self, _d: T) -> Self {
        unimplemented!();
    }

    /// Simplify using Visvalingam–Whyatt algorithm
    pub fn simplify_vw(&self, _d: T) -> Self {
        unimplemented!();
    }

    pub fn push(&mut self, point: nalgebra::Point2<T>) {
        self.points.push(point);
    }
}

impl<T> LineString3<T>
where
    T: nalgebra::Scalar + Copy + fmt::Debug + cmp::PartialOrd,
{
    pub fn default() -> Self {
        Self {
            points: Vec::<nalgebra::Point3<T>>::new(),
            connected: false,
        }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::<nalgebra::Point3<T>>::with_capacity(capacity),
            connected: false,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    pub fn as_lines(&self) -> Vec<Line3<T>> {
        if self.points.is_empty() {
            return vec![];
        } else if self.points.len() == 1 {
            return vec![Line3 {
                start: *self.points.first().unwrap(),
                end: *self.points.first().unwrap(),
            }];
        }
        let iter1 = self.points.iter().skip(1);
        let iter2 = self.points.iter();
        if self.connected {
            iter1
                .zip(iter2)
                .map(|(a, b)| Line3 { start: *b, end: *a })
                .chain(
                    Some(Line3 {
                        start: *self.points.first().unwrap(),
                        end: *self.points.last().unwrap(),
                    })
                    .into_iter(),
                )
                .collect()
        } else {
            iter1
                .zip(iter2)
                .map(|(a, b)| Line3 { start: *a, end: *b })
                .collect()
        }
    }

    pub fn push(&mut self, point: nalgebra::Point3<T>) {
        self.points.push(point);
    }
}

impl<T> LineStringSet2<T>
where
    T: nalgebra::Scalar + Copy + fmt::Debug + cmp::PartialOrd,
{
    pub fn default() -> Self {
        Self {
            set: Vec::<LineString2<T>>::new(),
            aabb: Aabb2::default(),
        }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            set: Vec::<LineString2<T>>::with_capacity(capacity),
            aabb: Aabb2::default(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.set.is_empty()
    }
}

impl<T> LineStringSet3<T>
where
    T: nalgebra::Scalar + Copy + fmt::Debug + cmp::PartialOrd,
{
    pub fn default() -> Self {
        Self {
            set: Vec::<LineString3<T>>::new(),
            aabb: Aabb3::default(),
        }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            set: Vec::<LineString3<T>>::with_capacity(capacity),
            aabb: Aabb3::default(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.set.is_empty()
    }

    pub fn push(&mut self, ls: LineString3<T>) {
        if !ls.is_empty() {
            self.set.push(ls);

            for ls in self.set.last().unwrap().points.iter() {
                self.aabb.update(ls);
            }
        }
    }

    /// Simplify using Ramer–Douglas–Peucker algorithm adapted for 3d
    pub fn simplify(&self, _d: T) -> Self {
        unimplemented!();
    }

    /// Simplify using Visvalingam–Whyatt algorithm adapted for 3d
    pub fn simplify_vw(&self, _d: T) -> Self {
        unimplemented!();
    }
}

impl<T> Aabb2<T>
where
    T: nalgebra::Scalar + Copy + fmt::Debug + cmp::PartialOrd,
{
    pub fn default() -> Self {
        Self {
            aabb_max: None,
            aabb_min: None,
        }
    }

    pub fn update(&mut self, point: &nalgebra::Point2<T>) {
        if self.aabb_max.is_none() {
            self.aabb_max = Some(*point);
        }
        if self.aabb_min.is_none() {
            self.aabb_min = Some(*point);
        }
        let mut aabb_min = self.aabb_min.take().unwrap();
        let mut aabb_max = self.aabb_max.take().unwrap();

        if point.x < aabb_min.x {
            aabb_min.x = point.x;
        }
        if point.y < aabb_min.y {
            aabb_min.y = point.y;
        }
        if point.x > aabb_max.x {
            aabb_max.x = point.x;
        }
        if point.y > aabb_max.y {
            aabb_max.y = point.y;
        }
        self.aabb_min = Some(aabb_min);
        self.aabb_max = Some(aabb_max);
    }

    pub fn get_high(&self) -> Option<nalgebra::Point2<T>> {
        self.aabb_max
    }

    pub fn get_low(&self) -> Option<nalgebra::Point2<T>> {
        self.aabb_min
    }
}

impl<T> Aabb3<T>
where
    T: nalgebra::Scalar + Copy + fmt::Debug + cmp::PartialOrd,
{
    pub fn default() -> Self {
        Self {
            aabb_max: None,
            aabb_min: None,
        }
    }

    pub fn update(&mut self, point: &nalgebra::Point3<T>) {
        if self.aabb_max.is_none() {
            self.aabb_max = Some(*point);
        }
        if self.aabb_min.is_none() {
            self.aabb_min = Some(*point);
        }
        let mut aabb_min = self.aabb_min.take().unwrap();
        let mut aabb_max = self.aabb_max.take().unwrap();

        if point.x < aabb_min.x {
            aabb_min.x = point.x;
        }
        if point.y < aabb_min.y {
            aabb_min.y = point.y;
        }
        if point.z < aabb_min.z {
            aabb_min.z = point.z;
        }
        if point.x > aabb_max.x {
            aabb_max.x = point.x;
        }
        if point.y > aabb_max.y {
            aabb_max.y = point.y;
        }
        if point.z > aabb_max.z {
            aabb_max.z = point.z;
        }
        self.aabb_min = Some(aabb_min);
        self.aabb_max = Some(aabb_max);
    }

    pub fn get_high(&self) -> Option<nalgebra::Point3<T>> {
        self.aabb_max
    }

    pub fn get_low(&self) -> Option<nalgebra::Point3<T>> {
        self.aabb_min
    }
}
