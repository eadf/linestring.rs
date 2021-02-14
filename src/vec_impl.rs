
use num_traits::Float;
use std::fmt;

/// Axis aligned planes, used to describe how imported 'flat' data is arranged in space
#[derive(fmt::Debug, Copy, Clone)]
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
        T: std::cmp::PartialOrd+ Float + approx::AbsDiffEq + approx::UlpsEq,
    {
        if let Some(low_bound) = aabb.get_low() {
            if let Some(high_bound) = aabb.get_high() {
                let dx = high_bound[0] - low_bound[0];
                let dy = high_bound[1] - low_bound[1];
                let dz = high_bound[2] - low_bound[2];
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
    T: std::cmp::PartialOrd+ Float,
{
    pub start: [T;2],
    pub end: [T;2],
}

impl<T> Line2<T>
where
    T: std::cmp::PartialOrd+ Float,
{
    pub fn new(start: [T;2], end: [T;2]) -> Self {
        Self { start, end }
    }
}

impl<T, IT> From<[IT; 2]> for Line2<T>
where
    T: std::cmp::PartialOrd+ Float,
    IT: Copy + Into<[T;2]>,
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
    T: std::cmp::PartialOrd+ Float,
{
    set: Vec<LineString2<T>>,
    aabb: Aabb2<T>,
}

/// A simple 2d AABB
/// If aabb_min_max is none the data has not been assigned yet.
#[derive(PartialEq, Eq, Clone, Hash, fmt::Debug)]
pub struct Aabb2<T>
where
    T: std::cmp::PartialOrd+ Float,
{
    aabb_min_max: Option<([T;2], [T;2])>,
}

#[derive(PartialEq, Eq, Clone, Hash, fmt::Debug)]
pub struct LineString2<T>
where
    T: std::cmp::PartialOrd+ Float,
{
    points: Vec<[T;2]>,

    /// if connected is set the as_lines() method will add an extra line connecting
    /// the first and last point
    pub connected: bool,
}

#[derive(PartialEq, Eq, Copy, Clone, Hash, fmt::Debug)]
pub struct Line3<T>
where
    T: std::cmp::PartialOrd+ Float,
{
    pub start: [T;3],
    pub end: [T;3],
}

impl<T> Line3<T>
where
    T: std::cmp::PartialOrd+ Float,
{
    pub fn new(start: [T;3], end: [T;3]) -> Self {
        Self { start, end }
    }
}

impl<T, IT> From<[IT; 2]> for Line3<T>
where
    T: std::cmp::PartialOrd+ Float,
    IT: Copy + Into<[T;3]>,
{
    fn from(pos: [IT; 2]) -> Line3<T> {
        Line3::<T>::new(pos[0].into(), pos[1].into())
    }
}

#[derive(PartialEq, Eq, Clone, Hash, fmt::Debug)]
pub struct LineString3<T>
where
    T: std::cmp::PartialOrd+ Float,
{
    points: Vec<[T;3]>,

    /// if connected is set the as_lines() method will add an extra line connecting
    /// the first and last point
    pub connected: bool,
}

/// A set of linestrings + an aabb
/// Intended to contain shapes with holes, e.g. outlines of letters
#[derive(PartialEq, Eq, Clone, Hash)]
pub struct LineStringSet3<T>
where
    T: std::cmp::PartialOrd+ Float,
{
    pub set: Vec<LineString3<T>>,
    pub aabb: Aabb3<T>,
}

#[derive(PartialEq, Eq, Copy, Clone, Hash, fmt::Debug)]
pub struct Aabb3<T>
where
    T: std::cmp::PartialOrd+ Float,
{
    aabb_min_max: Option<([T;3], [T;3])>,
}

impl<T> LineString2<T>
where
    T: std::cmp::PartialOrd+ Float,
{
    pub fn default() -> Self {
        Self {
            points: Vec::<[T;2]>::new(),
            connected: false,
        }
    }

    pub fn points(&self) -> &Vec<[T;2]> {
        &self.points
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::<[T;2]>::with_capacity(capacity),
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
                Plane::XY => [p2d[0],p2d[1],T::zero(),
               ],
                Plane::XZ => [p2d[0],T::zero(),p2d[1],
               ],
                Plane::ZY => [T::zero(),p2d[1],p2d[0],
               ],
            };
            rv.push(p3d);
        }
        rv
    }

    pub fn push(&mut self, point: [T;2]) {
        self.points.push(point);
    }

    #[cfg(not(feature="impl-vec"))]
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
    T: std::cmp::PartialOrd+ Float,
    IC: Into<[T;2]>,
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
    T: std::cmp::PartialOrd+ Float,
{
    pub fn default() -> Self {
        Self {
            points: Vec::<[T;3]>::new(),
            connected: false,
        }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::<[T;3]>::with_capacity(capacity),
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
                Plane::XY => [p3d[0],p3d[1]],
                Plane::XZ => [p3d[0],p3d[2]],
                Plane::ZY => [p3d[2],p3d[1]],
            };
            rv.push(p2d);
        }
        rv
    }

    pub fn points(&self) -> &Vec<[T;3]> {
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

    pub fn push(&mut self, point: [T;3]) {
        self.points.push(point);
    }

    #[cfg(not(feature="impl-vec"))]
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

impl<T, IC: Into<[T;3]>> std::iter::FromIterator<IC> for LineString3<T>
where
    T: std::cmp::PartialOrd+ Float,
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
    T: std::cmp::PartialOrd+ Float,
{
    pub fn default() -> Self {
        Self {
            set: Vec::<LineString2<T>>::new(),
            aabb: Aabb2::default(),
        }
    }

    pub fn set(&self) -> &Vec<LineString2<T>> {
        &self.set
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

    #[cfg(not(feature="impl-vec"))]
pub fn transform(&self, mat: &cgmath::Matrix3<T>) -> Self {
        Self {
            aabb: self.aabb.transform(mat),
            set: self.set.iter().map(|x| x.transform(mat)).collect(),
        }
    }

    /// Copy this linestringset2 into a linestringset3, populating the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    /// The empty axis will be set to zero.
    pub fn copy_to_3d(&self, plane: Plane) -> LineStringSet3<T> {
        let mut rv = LineStringSet3::with_capacity(self.set.len());
        for ls in self.set.iter() {
            rv.push(ls.copy_to_3d(plane));
        }
        rv
    }

    /// drains the 'other' container of all shapes and put them into 'self'
    pub fn take_from(&mut self, other: &mut Self) {
        self.aabb.update_aabb(&other.aabb);
        self.set.append(&mut other.set);
    }
}

impl<T> LineStringSet3<T>
where
    T: std::cmp::PartialOrd+ Float,
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

    pub fn set(&self) -> &Vec<LineString3<T>> {
        &self.set
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

    #[cfg(not(feature="impl-vec"))]
pub fn transform(&self, mat: &cgmath::Matrix4<T>) -> Self {
        Self {
            set: self.set.iter().map(|x| x.transform(mat)).collect(),
            aabb: self.aabb.transform(mat),
        }
    }

    /// Copy this linestringset3 into a linestringset2, populating the axes defined by 'plane'
    /// An axis will always try to keep it's position (e.g. y goes to y if possible).
    /// That way the operation is reversible (with regards to axis positions).
    pub fn copy_to_2d(&self, plane: Plane) -> LineStringSet2<T> {
        let mut rv = LineStringSet2::with_capacity(self.set.len());
        for ls in self.set.iter() {
            rv.push(ls.copy_to_2d(plane));
        }
        rv
    }

    /// drains the 'other' container of all shapes and put them into 'self'
    pub fn take_from(&mut self, other: &mut Self) {
        self.aabb.update_aabb(&other.aabb);
        self.set.append(&mut other.set);
    }
}

impl<T> Aabb2<T>
where
    T: std::cmp::PartialOrd+ Float,
{
    pub fn default() -> Self {
        Self { aabb_min_max: None }
    }

    pub fn new(point: &[T;2]) -> Self {
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

    pub fn update_point(&mut self, point: &[T;2]) {
        if self.aabb_min_max.is_none() {
            self.aabb_min_max = Some((*point, *point));
        }
        let (mut aabb_min, mut aabb_max) = self.aabb_min_max.take().unwrap();

        if point[0] < aabb_min[0] {
            aabb_min[0] = point[0];
        }
        if point[1] < aabb_min[1] {
            aabb_min[1] = point[1];
        }
        if point[0] > aabb_max[0] {
            aabb_max[0] = point[0];
        }
        if point[1] > aabb_max[1] {
            aabb_max[1] = point[1];
        }
        self.aabb_min_max = Some((aabb_min, aabb_max));
    }

    pub fn get_high(&self) -> Option<[T;2]> {
        if let Some((_, _high)) = self.aabb_min_max {
            return Some(_high);
        }
        None
    }

    pub fn get_low(&self) -> Option<[T;2]> {
        if let Some((_low, _)) = self.aabb_min_max {
            return Some(_low);
        }
        None
    }

    #[cfg(not(feature="impl-vec"))]
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

    /// returns true if this aabb contains 'other' (inclusive)
    #[inline(always)]
    pub fn contains_aabb(&self, other: &Aabb2<T>) -> bool {
        if let Some(o_aabb) = other.aabb_min_max {
            return self.contains_point(&o_aabb.0) && self.contains_point(&o_aabb.1);
        }
        false
    }

    /// returns true if this aabb contains a point (inclusive)
    #[inline(always)]
    pub fn contains_line(&self, line: &Line2<T>) -> bool {
        self.contains_point(&line.start) && self.contains_point(&line.end)
    }

    /// returns true if this aabb contains a point (inclusive)
    #[inline(always)]
    pub fn contains_point(&self, point: &[T;2]) -> bool {
        if let Some(s_aabb) = self.aabb_min_max {
            return s_aabb.0[0] <= point[0]
                && s_aabb.0[1] <= point[1]
                && s_aabb.1[0] >= point[0]
                && s_aabb.1[1] >= point[1];
        }
        false
    }
}

impl<T> Aabb3<T>
where
    T: std::cmp::PartialOrd+ Float,
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

    pub fn update_point(&mut self, point: &[T;3]) {
        if self.aabb_min_max.is_none() {
            self.aabb_min_max = Some((*point, *point));
        }
        let (mut aabb_min, mut aabb_max) = self.aabb_min_max.take().unwrap();

        if point[0] < aabb_min[0] {
            aabb_min[0] = point[0];
        }
        if point[1] < aabb_min[1] {
            aabb_min[1] = point[1];
        }
        if point[2] < aabb_min[2] {
            aabb_min[2] = point[2];
        }
        if point[0] > aabb_max[0] {
            aabb_max[0] = point[0];
        }
        if point[1] > aabb_max[1] {
            aabb_max[1] = point[1];
        }
        if point[2] > aabb_max[2] {
            aabb_max[2] = point[2];
        }
        self.aabb_min_max = Some((aabb_min, aabb_max));
    }

    pub fn get_high(&self) -> Option<[T;3]> {
        if let Some((_, _high)) = self.aabb_min_max {
            return Some(_high);
        }
        None
    }

    pub fn get_low(&self) -> Option<[T;3]> {
        if let Some((_low, _)) = self.aabb_min_max {
            return Some(_low);
        }
        None
    }

    #[cfg(not(feature="impl-vec"))]
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

    /// returns true if this aabb contains 'other' (inclusive)
    #[inline(always)]
    pub fn contains_aabb(&self, other: &Aabb3<T>) -> bool {
        if let Some(o_aabb) = other.aabb_min_max {
            return self.contains_point(&o_aabb.0) && self.contains_point(&o_aabb.1);
        }
        false
    }

    /// returns true if this aabb contains a point (inclusive)
    #[inline(always)]
    pub fn contains_line(&self, line: &Line3<T>) -> bool {
        self.contains_point(&line.start) && self.contains_point(&line.end)
    }

    /// returns true if this aabb contains a point (inclusive)
    #[inline(always)]
    pub fn contains_point(&self, point: &[T;3]) -> bool {
        if let Some(s_aabb) = self.aabb_min_max {
            return s_aabb.0[0] <= point[0]
                && s_aabb.0[1] <= point[1]
                && s_aabb.0[2] <= point[2]
                && s_aabb.1[0] >= point[0]
                && s_aabb.1[1] >= point[1]
                && s_aabb.1[2] >= point[2];
        }
        false
    }
}
