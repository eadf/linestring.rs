
use num_traits::{Float, Zero};
use std::cmp;
use std::fmt;
use std::ops;

// Auto generated file, do NOT edit

/// Axis aligned planes, used to describe how imported 'flat' data is arranged in space
#[derive(fmt::Debug)]
pub enum Plane {
    XY,
    XZ,
    ZY,
}

impl Plane {
    pub fn get_plane<T>(aabb: &Aabb3<T>) -> Option<Plane>
    where
        T: Copy
            + Float
            + nalgebra::Scalar + nalgebra::RealField
            + fmt::Debug
            + cmp::PartialOrd
            + ops::Sub<Output = T>
            + Zero
            + approx::AbsDiffEq
            + approx::UlpsEq,
    {
        if let Some(lowb) = aabb.get_low() {
            if let Some(highb) = aabb.get_high() {
                let dx = T::zero().ulps_eq(
                    &(highb.x - lowb.x),
                    T::default_epsilon(),
                    T::default_max_ulps(),
                );
                let dy = T::zero().ulps_eq(
                    &(highb.y - lowb.y),
                    T::default_epsilon(),
                    T::default_max_ulps(),
                );
                let dz = T::zero().ulps_eq(
                    &(highb.z - lowb.z),
                    T::default_epsilon(),
                    T::default_max_ulps(),
                );

                if dx && !dy && !dz {
                    return Some(Plane::XY);
                }
                if dy && !dx && !dz {
                    return Some(Plane::XZ);
                }
                if dz && !dx && !dy {
                    return Some(Plane::ZY);
                }
            }
        }
        None
    }
}

#[derive(PartialEq, Eq, Copy, Clone, Hash, fmt::Debug)]
pub struct Line2<T>
where
    T: Copy + nalgebra::Scalar + nalgebra::RealField + fmt::Debug + cmp::PartialOrd,
{
    pub start: nalgebra::Point2<T>,
    pub end: nalgebra::Point2<T>,
}

/// A set of linestrings + an aabb
/// Intended to contain shapes with holes, e.g. outlines of letters
#[derive(PartialEq, Eq, Clone, Hash)]
pub struct LineStringSet2<T>
where
    T: Copy + nalgebra::Scalar + nalgebra::RealField + fmt::Debug + cmp::PartialOrd,
{
    set: Vec<LineString2<T>>,
    aabb: Aabb2<T>,
}

#[derive(PartialEq, Eq, Clone, Hash, fmt::Debug)]
pub struct Aabb2<T>
where
    T: Copy + nalgebra::Scalar + nalgebra::RealField + fmt::Debug + cmp::PartialOrd,
{
    aabb_min_max: Option<(nalgebra::Point2<T>, nalgebra::Point2<T>)>,
}

#[derive(PartialEq, Eq, Clone, Hash, fmt::Debug)]
pub struct LineString2<T>
where
    T: Copy + nalgebra::Scalar + nalgebra::RealField + fmt::Debug + cmp::PartialOrd,
{
    points: Vec<nalgebra::Point2<T>>,
    pub connected: bool,
}

#[derive(PartialEq, Eq, Copy, Clone, Hash, fmt::Debug)]
pub struct Line3<T>
where
    T: Copy + nalgebra::Scalar + nalgebra::RealField + fmt::Debug + cmp::PartialOrd,
{
    pub start: nalgebra::Point3<T>,
    pub end: nalgebra::Point3<T>,
}

#[derive(PartialEq, Eq, Clone, Hash, fmt::Debug)]
pub struct LineString3<T>
where
    T: Copy + nalgebra::Scalar + nalgebra::RealField + fmt::Debug + cmp::PartialOrd,
{
    points: Vec<nalgebra::Point3<T>>,
    // would name this 'loop' but it's reserved
    pub connected: bool,
}

/// A set of linestrings + an aabb
/// Intended to contain shapes with holes, e.g. outlines of letters
#[derive(PartialEq, Eq, Clone, Hash)]
pub struct LineStringSet3<T>
where
    T: Copy + nalgebra::Scalar + nalgebra::RealField + fmt::Debug + cmp::PartialOrd,
{
    pub set: Vec<LineString3<T>>,
    pub aabb: Aabb3<T>,
}

#[derive(PartialEq, Eq, Copy, Clone, Hash, fmt::Debug)]
pub struct Aabb3<T>
where
    T: Copy + nalgebra::Scalar + nalgebra::RealField + fmt::Debug + cmp::PartialOrd,
{
    aabb_min_max: Option<(nalgebra::Point3<T>, nalgebra::Point3<T>)>,
}

impl<T> LineString2<T>
where
    T: Copy + nalgebra::Scalar + nalgebra::RealField + fmt::Debug + cmp::PartialOrd,
{
    pub fn default() -> Self {
        Self {
            points: Vec::<nalgebra::Point2<T>>::new(),
            connected: false,
        }
    }

    pub fn points(&self) -> &Vec<nalgebra::Point2<T>> {
        &self.points
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::<nalgebra::Point2<T>>::with_capacity(capacity),
            connected: false,
        }
    }

    pub fn len(&self) -> usize {
        self.points.len()
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
        if self.connected && self.points.last() != self.points.first() {
            iter1
                .zip(iter2)
                .map(|(a, b)| Line2 { start: *b, end: *a })
                .chain(
                    Some(Line2 {
                        start: *self.points.last().unwrap(),
                        end: *self.points.first().unwrap(),
                    })
                    .into_iter(),
                )
                .collect()
        } else {
            iter1
                .zip(iter2)
                .map(|(a, b)| Line2 { start: *b, end: *a })
                .collect()
        }
    }

    pub fn push(&mut self, point: nalgebra::Point2<T>) {
        self.points.push(point);
    }

    //#[cfg(not(feature="impl-mint"))]
    pub fn transform(&self, mat: &nalgebra::Matrix3<T>) -> Self {
        Self {
            points: self
                .points
                .iter()
                .map(|x| mat.transform_point(x))
                .collect(),
            connected: self.connected,
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
}
impl<T, IC: Into<nalgebra::Point2<T>>> std::iter::FromIterator<IC> for LineString2<T>
where
    T: Copy + nalgebra::Scalar + nalgebra::RealField + fmt::Debug + cmp::PartialOrd,
{
    fn from_iter<I: IntoIterator<Item = IC>>(iter: I) -> Self {
        LineString2 {
            points: iter.into_iter().map(|c| c.into()).collect(),
            connected: false,
        }
    }
}

impl<T> LineString3<T>
where
    T: Copy + nalgebra::Scalar + nalgebra::RealField + fmt::Debug + cmp::PartialOrd,
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

    pub fn points(&self) -> &Vec<nalgebra::Point3<T>> {
        &self.points
    }

    pub fn len(&self) -> usize {
        self.points.len()
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
        if self.connected && self.points.last() != self.points.first() {
            iter1
                .zip(iter2)
                .map(|(a, b)| Line3 { start: *b, end: *a })
                .chain(
                    Some(Line3 {
                        start: *self.points.last().unwrap(),
                        end: *self.points.first().unwrap(),
                    })
                    .into_iter(),
                )
                .collect()
        } else {
            iter1
                .zip(iter2)
                .map(|(a, b)| Line3 { start: *b, end: *a })
                .collect()
        }
    }

    pub fn push(&mut self, point: nalgebra::Point3<T>) {
        self.points.push(point);
    }

    //#[cfg(not(feature="impl-mint"))]
    pub fn transform(&self, mat: &nalgebra::Matrix4<T>) -> Self {
        Self {
            points: self
                .points
                .iter()
                .map(|x| mat.transform_point(x))
                .collect(),
            connected: self.connected,
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

impl<T, IC: Into<nalgebra::Point3<T>>> std::iter::FromIterator<IC> for LineString3<T>
where
    T: Copy + nalgebra::Scalar + nalgebra::RealField + fmt::Debug + cmp::PartialOrd,
{
    fn from_iter<I: IntoIterator<Item = IC>>(iter: I) -> Self {
        LineString3 {
            points: iter.into_iter().map(|c| c.into()).collect(),
            connected: false,
        }
    }
}

impl<T> LineStringSet2<T>
where
    T: Copy + nalgebra::Scalar + nalgebra::RealField + fmt::Debug + cmp::PartialOrd,
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

    pub fn push(&mut self, ls: LineString2<T>) {
        if !ls.is_empty() {
            self.set.push(ls);

            for ls in self.set.last().unwrap().points.iter() {
                self.aabb.update_point(ls);
            }
        }
    }

    pub fn get_aabb(&self) -> &Aabb2<T> {
        &self.aabb
    }

    //#[cfg(not(feature="impl-mint"))]
    pub fn transform(&self, mat: &nalgebra::Matrix3<T>) -> Self {
        Self {
            aabb: self.aabb.transform(mat),
            set: self.set.iter().map(|x| x.transform(mat)).collect(),
        }
    }
}

impl<T> LineStringSet3<T>
where
    T: Copy + nalgebra::Scalar + nalgebra::RealField + fmt::Debug + cmp::PartialOrd,
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
                self.aabb.update_point(ls);
            }
        }
    }

    pub fn get_aabb(&self) -> &Aabb3<T> {
        &self.aabb
    }

    //#[cfg(not(feature="impl-mint"))]
    pub fn transform(&self, mat: &nalgebra::Matrix4<T>) -> Self {
        Self {
            set: self.set.iter().map(|x| x.transform(mat)).collect(),
            aabb: self.aabb.transform(mat),
        }
    }
}

impl<T> Aabb2<T>
where
    T: Copy + nalgebra::Scalar + nalgebra::RealField + fmt::Debug + cmp::PartialOrd,
{
    pub fn default() -> Self {
        Self { aabb_min_max: None }
    }

    pub fn new(point: &nalgebra::Point2<T>) -> Self {
        Self {
            aabb_min_max: Some((*point, *point)),
        }
    }

    pub fn update_aabb(&mut self, aabb: &Aabb2<T>) {
        if let Some((min, max)) = &aabb.aabb_min_max {
            self.update_point(min);
            self.update_point(max);
        }
    }

    pub fn update_point(&mut self, point: &nalgebra::Point2<T>) {
        if self.aabb_min_max.is_none() {
            self.aabb_min_max = Some((*point, *point));
        }
        let (mut aabb_min, mut aabb_max) = self.aabb_min_max.take().unwrap();

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
        self.aabb_min_max = Some((aabb_min, aabb_max));
    }

    pub fn get_high(&self) -> Option<nalgebra::Point2<T>> {
        if let Some((_, _high)) = self.aabb_min_max {
            return Some(_high);
        }
        None
    }

    pub fn get_low(&self) -> Option<nalgebra::Point2<T>> {
        if let Some((_low, _)) = self.aabb_min_max {
            return Some(_low);
        }
        None
    }

    //#[cfg(not(feature="impl-mint"))]
    pub fn transform(&self, mat: &nalgebra::Matrix3<T>) -> Self {
        if let Some(aabb_min_max) = self.aabb_min_max {
            Self {
                aabb_min_max: Some((
                    mat.transform_point(&aabb_min_max.0),
                    mat.transform_point(&aabb_min_max.1),
                )),
            }
        } else {
            Self { aabb_min_max: None }
        }
    }
}

impl<T> Aabb3<T>
where
    T: Copy + nalgebra::Scalar + nalgebra::RealField + fmt::Debug + cmp::PartialOrd,
{
    pub fn default() -> Self {
        Self { aabb_min_max: None }
    }

    pub fn update_aabb(&mut self, aabb: &Aabb3<T>) {
        if let Some((min, max)) = &aabb.aabb_min_max {
            self.update_point(min);
            self.update_point(max);
        }
    }

    pub fn update_point(&mut self, point: &nalgebra::Point3<T>) {
        if self.aabb_min_max.is_none() {
            self.aabb_min_max = Some((*point, *point));
        }
        let (mut aabb_min, mut aabb_max) = self.aabb_min_max.take().unwrap();

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
        self.aabb_min_max = Some((aabb_min, aabb_max));
    }

    pub fn get_high(&self) -> Option<nalgebra::Point3<T>> {
        if let Some((_, _high)) = self.aabb_min_max {
            return Some(_high);
        }
        None
    }

    pub fn get_low(&self) -> Option<nalgebra::Point3<T>> {
        if let Some((_low, _)) = self.aabb_min_max {
            return Some(_low);
        }
        None
    }

    //#[cfg(not(feature="impl-mint"))]
    pub fn transform(&self, mat: &nalgebra::Matrix4<T>) -> Self {
        if let Some(aabb_min_max) = self.aabb_min_max {
            Self {
                aabb_min_max: Some((
                    mat.transform_point(&aabb_min_max.0),
                    mat.transform_point(&aabb_min_max.1),
                )),
            }
        } else {
            Self { aabb_min_max: None }
        }
    }
}
