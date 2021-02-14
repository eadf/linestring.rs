use cgmath::Transform;
use num_traits::Float;
use std::fmt;

// Original file, edit this one and convert to the others

/// Axis aligned planes, used to describe how imported 'flat' data is arranged in space
#[derive(fmt::Debug)]
pub enum Plane {
    XY,
    XZ,
    ZY,
}

impl Plane {
    /// Try to figure out what axes defines the plane.
    /// If the AABB delta of one axis (a) is virtually nothing compared to
    /// the widest axis (b) while the third axis (c) is comparable to (b)
    /// by some fraction, we assume that that (a) isn't part of the plane.
    ///
    /// It's not possible to compare to zero exactly because blender
    /// leaves some decimal in coordinates that's suppose to be zero.
    pub fn get_plane<T>(aabb: &Aabb3<T>) -> Option<Plane>
    where
        T: cgmath::BaseFloat + approx::AbsDiffEq + approx::UlpsEq,
    {
        if let Some(low_bound) = aabb.get_low() {
            if let Some(high_bound) = aabb.get_high() {
                let dx = high_bound.x - low_bound.x;
                let dy = high_bound.y - low_bound.y;
                let dz = high_bound.z - low_bound.z;
                let max_delta = Float::max(Float::max(dx, dy), dz);

                let dx = T::zero().ulps_eq(
                    &(dx / max_delta),
                    T::default_epsilon(),
                    T::default_max_ulps(),
                );
                let dy = T::zero().ulps_eq(
                    &&(dy / max_delta),
                    T::default_epsilon(),
                    T::default_max_ulps(),
                );
                let dz = T::zero().ulps_eq(
                    &&(dz / max_delta),
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
    T: cgmath::BaseFloat,
{
    pub start: cgmath::Point2<T>,
    pub end: cgmath::Point2<T>,
}

impl<T> Line2<T>
where
    T: cgmath::BaseFloat,
{
    pub fn new(start: cgmath::Point2<T>, end: cgmath::Point2<T>) -> Self {
        Self { start, end }
    }
}

impl<T, IT> From<[IT; 2]> for Line2<T>
where
    T: cgmath::BaseFloat,
    IT: Copy + Into<cgmath::Point2<T>>,
{
    fn from(coordinate: [IT; 2]) -> Line2<T> {
        Line2::<T>::new(coordinate[0].into(), coordinate[1].into())
    }
}

/// A set of linestrings + an aabb
/// Intended to contain related shapes. E.g. outlines of letters with holes
#[derive(PartialEq, Eq, Clone, Hash)]
pub struct LineStringSet2<T>
where
    T: cgmath::BaseFloat,
{
    set: Vec<LineString2<T>>,
    aabb: Aabb2<T>,
}

/// A simple 2d AABB
/// If aabb_min_max is none the data has not been assigned yet.
#[derive(PartialEq, Eq, Clone, Hash, fmt::Debug)]
pub struct Aabb2<T>
where
    T: cgmath::BaseFloat,
{
    aabb_min_max: Option<(cgmath::Point2<T>, cgmath::Point2<T>)>,
}

#[derive(PartialEq, Eq, Clone, Hash, fmt::Debug)]
pub struct LineString2<T>
where
    T: cgmath::BaseFloat,
{
    points: Vec<cgmath::Point2<T>>,

    /// if connected is set the as_lines() method will add an extra line connecting
    /// the first and last point
    pub connected: bool,
}

#[derive(PartialEq, Eq, Copy, Clone, Hash, fmt::Debug)]
pub struct Line3<T>
where
    T: cgmath::BaseFloat,
{
    pub start: cgmath::Point3<T>,
    pub end: cgmath::Point3<T>,
}

impl<T> Line3<T>
where
    T: cgmath::BaseFloat,
{
    pub fn new(start: cgmath::Point3<T>, end: cgmath::Point3<T>) -> Self {
        Self { start, end }
    }
}

impl<T, IT> From<[IT; 2]> for Line3<T>
where
    T: cgmath::BaseFloat,
    IT: Copy + Into<cgmath::Point3<T>>,
{
    fn from(pos: [IT; 2]) -> Line3<T> {
        Line3::<T>::new(pos[0].into(), pos[1].into())
    }
}

#[derive(PartialEq, Eq, Clone, Hash, fmt::Debug)]
pub struct LineString3<T>
where
    T: cgmath::BaseFloat,
{
    points: Vec<cgmath::Point3<T>>,

    /// if connected is set the as_lines() method will add an extra line connecting
    /// the first and last point
    pub connected: bool,
}

/// A set of linestrings + an aabb
/// Intended to contain shapes with holes, e.g. outlines of letters
#[derive(PartialEq, Eq, Clone, Hash)]
pub struct LineStringSet3<T>
where
    T: cgmath::BaseFloat,
{
    pub set: Vec<LineString3<T>>,
    pub aabb: Aabb3<T>,
}

#[derive(PartialEq, Eq, Copy, Clone, Hash, fmt::Debug)]
pub struct Aabb3<T>
where
    T: cgmath::BaseFloat,
{
    aabb_min_max: Option<(cgmath::Point3<T>, cgmath::Point3<T>)>,
}

impl<T> LineString2<T>
where
    T: cgmath::BaseFloat,
{
    pub fn default() -> Self {
        Self {
            points: Vec::<cgmath::Point2<T>>::new(),
            connected: false,
        }
    }

    pub fn points(&self) -> &Vec<cgmath::Point2<T>> {
        &self.points
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::<cgmath::Point2<T>>::with_capacity(capacity),
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

    /// Copy this linestring2 into a linestring3, populating the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    pub fn copy_to_3d(&self, plane: Plane) -> LineString3<T> {
        let mut rv = LineString3::<T>::with_capacity(self.len());
        for p2d in self.points.iter() {
            let p3d = match plane {
                Plane::XY => cgmath::Point3 {
                    x: p2d.x,
                    y: p2d.y,
                    z: T::zero(),
                },
                Plane::XZ => cgmath::Point3 {
                    x: p2d.x,
                    y: T::zero(),
                    z: p2d.y,
                },
                Plane::ZY => cgmath::Point3 {
                    x: T::zero(),
                    y: p2d.y,
                    z: p2d.x,
                },
            };
            rv.push(p3d);
        }
        rv
    }

    pub fn push(&mut self, point: cgmath::Point2<T>) {
        self.points.push(point);
    }

    pub fn transform(&self, mat: &cgmath::Matrix3<T>) -> Self {
        Self {
            points: self
                .points
                .iter()
                .map(|x| mat.transform_point(*x))
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

impl<T, IC> std::iter::FromIterator<IC> for LineString2<T>
where
    T: cgmath::BaseFloat,
    IC: Into<cgmath::Point2<T>>,
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
    T: cgmath::BaseFloat,
{
    pub fn default() -> Self {
        Self {
            points: Vec::<cgmath::Point3<T>>::new(),
            connected: false,
        }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::<cgmath::Point3<T>>::with_capacity(capacity),
            connected: false,
        }
    }

    /// Copy this linestring3 into a linestring2, keeping the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    pub fn copy_to_2d(&self, plane: Plane) -> LineString2<T> {
        let mut rv = LineString2::<T>::with_capacity(self.len());
        for p3d in self.points.iter() {
            let p2d = match plane {
                Plane::XY => cgmath::Point2 { x: p3d.x, y: p3d.y },
                Plane::XZ => cgmath::Point2 { x: p3d.x, y: p3d.z },
                Plane::ZY => cgmath::Point2 { x: p3d.z, y: p3d.y },
            };
            rv.push(p2d);
        }
        rv
    }

    pub fn points(&self) -> &Vec<cgmath::Point3<T>> {
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

    pub fn push(&mut self, point: cgmath::Point3<T>) {
        self.points.push(point);
    }

    pub fn transform(&self, mat: &cgmath::Matrix4<T>) -> Self {
        Self {
            points: self
                .points
                .iter()
                .map(|x| mat.transform_point(*x))
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

impl<T, IC: Into<cgmath::Point3<T>>> std::iter::FromIterator<IC> for LineString3<T>
where
    T: cgmath::BaseFloat,
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
    T: cgmath::BaseFloat,
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

    pub fn transform(&self, mat: &cgmath::Matrix3<T>) -> Self {
        Self {
            aabb: self.aabb.transform(mat),
            set: self.set.iter().map(|x| x.transform(mat)).collect(),
        }
    }
}

impl<T> LineStringSet3<T>
where
    T: cgmath::BaseFloat,
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

    pub fn transform(&self, mat: &cgmath::Matrix4<T>) -> Self {
        Self {
            set: self.set.iter().map(|x| x.transform(mat)).collect(),
            aabb: self.aabb.transform(mat),
        }
    }
}

impl<T> Aabb2<T>
where
    T: cgmath::BaseFloat,
{
    pub fn default() -> Self {
        Self { aabb_min_max: None }
    }

    pub fn new(point: &cgmath::Point2<T>) -> Self {
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

    pub fn update_point(&mut self, point: &cgmath::Point2<T>) {
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

    pub fn get_high(&self) -> Option<cgmath::Point2<T>> {
        if let Some((_, _high)) = self.aabb_min_max {
            return Some(_high);
        }
        None
    }

    pub fn get_low(&self) -> Option<cgmath::Point2<T>> {
        if let Some((_low, _)) = self.aabb_min_max {
            return Some(_low);
        }
        None
    }

    pub fn transform(&self, mat: &cgmath::Matrix3<T>) -> Self {
        if let Some(aabb_min_max) = self.aabb_min_max {
            Self {
                aabb_min_max: Some((
                    mat.transform_point(aabb_min_max.0),
                    mat.transform_point(aabb_min_max.1),
                )),
            }
        } else {
            Self { aabb_min_max: None }
        }
    }
}

impl<T> Aabb3<T>
where
    T: cgmath::BaseFloat,
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

    pub fn update_point(&mut self, point: &cgmath::Point3<T>) {
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

    pub fn get_high(&self) -> Option<cgmath::Point3<T>> {
        if let Some((_, _high)) = self.aabb_min_max {
            return Some(_high);
        }
        None
    }

    pub fn get_low(&self) -> Option<cgmath::Point3<T>> {
        if let Some((_low, _)) = self.aabb_min_max {
            return Some(_low);
        }
        None
    }

    pub fn transform(&self, mat: &cgmath::Matrix4<T>) -> Self {
        if let Some(aabb_min_max) = self.aabb_min_max {
            Self {
                aabb_min_max: Some((
                    mat.transform_point(aabb_min_max.0),
                    mat.transform_point(aabb_min_max.1),
                )),
            }
        } else {
            Self { aabb_min_max: None }
        }
    }
}
