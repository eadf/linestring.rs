// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2021,2023 lacklustr@protonmail.com https://github.com/eadf

// This file is part of the linestring crate.

/*
Copyright (c) 2021,2023 lacklustr@protonmail.com https://github.com/eadf

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

or

Copyright 2021,2023 lacklustr@protonmail.com https://github.com/eadf

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

use crate::{linestring_2d, LinestringError};
use ahash::AHashSet;
use std::{cmp::Ordering, collections::BTreeMap, convert::identity, fmt, fmt::Debug};
use vector_traits::{approx::*, num_traits::float::FloatCore, GenericScalar, GenericVector2};

#[derive(Clone, Copy)]
pub struct SiteEventKey<T: GenericVector2> {
    pub pos: T,
}

impl<T: GenericVector2> SiteEventKey<T> {
    pub fn new(x: T::Scalar, y: T::Scalar) -> Self {
        Self {
            pos: T::new_2d(x, y),
        }
    }
}

impl<T: GenericVector2> Debug for SiteEventKey<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("")
            .field(&self.pos.x())
            .field(&self.pos.y())
            .finish()
    }
}

impl<T: GenericVector2> Ord for SiteEventKey<T> {
    /// It should be impossible for a !is_finite() number to be added to SiteEventKey
    #[inline(always)]
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(other).unwrap()
    }
}

#[allow(clippy::non_canonical_partial_ord_impl)]
impl<T: GenericVector2> PartialOrd for SiteEventKey<T>
where
    T::Scalar: UlpsEq,
{
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        if ulps_eq!(&self.pos.y(), &other.pos.y()) {
            if ulps_eq!(&self.pos.x(), &other.pos.x()) {
                Some(Ordering::Equal)
            } else {
                self.pos.x().partial_cmp(&other.pos.x())
            }
        } else {
            self.pos.y().partial_cmp(&other.pos.y())
        }
    }
}

impl<T: GenericVector2> Eq for SiteEventKey<T> {}

impl<T: GenericVector2> PartialEq for SiteEventKey<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.pos.is_ulps_eq(
            other.pos,
            T::Scalar::default_epsilon(),
            T::Scalar::default_max_ulps(),
        )
    }
}

/// A container struct that keeps track of the lines around a pivot point.
/// It only stores the lines with highest x value left of pivot point, and lines with lowest x
/// value right of the point. Secondarily it prioritizes according to slope of the line, lines
/// leaning towards pivot point have priority.
struct MinMax<T: GenericVector2> {
    best_left: Option<T::Scalar>,
    slope: MinMaxSlope<T>,
    best_right: Option<T::Scalar>,
}

impl<T: GenericVector2> MinMax<T>
where
    T::Scalar: UlpsEq,
{
    fn new() -> Self {
        Self {
            best_left: None,
            best_right: None,
            slope: MinMaxSlope::new(),
        }
    }

    /// keep track of the candidates closest (on both sides) to pivot_x
    fn update(
        &mut self,
        pivot_x: T::Scalar,
        candidate_x: T::Scalar,
        candidate_slope: T::Scalar,
        candidate_index: usize,
    ) {
        if candidate_x < pivot_x {
            /*println!(
                "Left:looking at {} candidate_x:{} candidate_slope:{}",
                candidate_index, candidate_x, candidate_slope
            );*/
            // handle left side
            if let Some(current_min) = self.best_left {
                if ulps_eq!(&current_min, &candidate_x) {
                    self.slope
                        .update_left(false, candidate_slope, candidate_index);
                } else if current_min < candidate_x {
                    // this candidate is better.
                    self.slope
                        .update_left(true, candidate_slope, candidate_index);
                    self.best_left = Some(candidate_x);
                } else {
                    /*println!(
                        "Left2:rejecting {} candidate_x:{} current_x:{} candidate_slope:{}, current_slope:{}",
                        candidate_index, candidate_x, current_min, candidate_slope, self.slope.best_left.unwrap()
                    )*/
                }
            } else {
                // First sample
                self.best_left = Some(candidate_x);
                self.slope
                    .update_left(false, candidate_slope, candidate_index);
            }
        } else if candidate_x > pivot_x {
            /*println!(
                "Right:looking at {} candidate_x:{} candidate_slope:{}",
                candidate_index, candidate_x, candidate_slope
            );*/
            // handle right side
            if let Some(current_max) = self.best_right {
                if ulps_eq!(&current_max, &candidate_x) {
                    self.slope
                        .update_right(false, candidate_slope, candidate_index);
                } else if current_max > candidate_x {
                    // this candidate is better.
                    self.slope
                        .update_right(true, candidate_slope, candidate_index);
                    self.best_right = Some(candidate_x);
                } else {
                    /*println!(
                        "Right2:rejecting {} candidate_x:{} current_x:{} candidate_slope:{}, current_slope:{}",
                        candidate_index, candidate_x, current_max, candidate_slope, self.slope.best_right.unwrap()
                    )*/
                }
            } else {
                // First sample
                self.best_right = Some(candidate_x);
                self.slope
                    .update_right(false, candidate_slope, candidate_index);
            }
        }
    }

    /// clear all data
    fn clear(&mut self) {
        self.best_left = None;
        self.best_right = None;
        self.slope.clear();
    }
}

struct MinMaxSlope<T: GenericVector2> {
    best_left: Option<T::Scalar>, // slope
    candidates_left: Vec<usize>,

    best_right: Option<T::Scalar>, // slope
    candidates_right: Vec<usize>,
}

impl<T: GenericVector2> MinMaxSlope<T>
where
    T::Scalar: UlpsEq,
{
    fn new() -> Self {
        Self {
            best_left: None,
            candidates_left: Vec::<usize>::new(),
            best_right: None,
            candidates_right: Vec::<usize>::new(),
        }
    }

    /// sort candidates based on slope, keep only the ones with 'flattest' angle to the left and right
    fn update_both(&mut self, candidate_index: usize, lines: &[linestring_2d::Line2<T>]) {
        let line = lines[candidate_index];
        let candidate_slope = if ulps_eq!(&line.end.y(), &line.start.y()) {
            T::Scalar::INFINITY
        } else {
            (line.end.x() - line.start.x()) / (line.end.y() - line.start.y())
        };

        self.update_left(false, candidate_slope, candidate_index);
        self.update_right(false, candidate_slope, candidate_index);
    }

    /// sort candidates based on slope, keep only the best to the left -> lowest abs(negative slope value)
    fn update_left(&mut self, clear: bool, candidate_slope: T::Scalar, candidate_index: usize) {
        if clear {
            self.candidates_left.clear();
            self.best_left = None;
        }
        /*println!(
            "Left:looking at {} candidate_slope:{}",
            candidate_index, candidate_slope
        );*/
        // handle left side
        if let Some(current_slope) = self.best_left {
            if ulps_eq!(&current_slope, &candidate_slope) {
                // this candidate is just as good as the others already found
                self.candidates_left.push(candidate_index);
            } else if candidate_slope < current_slope {
                // this candidate is better.
                self.candidates_left.clear();
                self.candidates_left.push(candidate_index);
                self.best_left = Some(candidate_slope);
            } else {
                /*println!(
                    "Left1:rejecting {} candidate_slope:{}, current_slope:{}",
                    candidate_index, candidate_slope, current_slope
                )*/
            }
        } else {
            // First sample
            self.best_left = Some(candidate_slope);
            self.candidates_left.push(candidate_index);
        }
        /*println!(
            "Left: best slope is {:?} ,best_left:{}",
            self.candidates_left,
            self.best_left.unwrap()
        )*/
    }

    /// sort candidates based on slope, keep only the best to the left -> lowest positive slope value
    fn update_right(&mut self, clear: bool, candidate_slope: T::Scalar, candidate_index: usize) {
        /*println!(
            "Right:looking at {} candidate_slope:{}",
            candidate_index, candidate_slope
        );*/
        if clear {
            self.candidates_right.clear();
            self.best_right = None;
        }
        // handle right side
        if let Some(current_slope) = self.best_right {
            if ulps_eq!(&current_slope, &candidate_slope) {
                // this candidate is just as good as the others already found
                self.candidates_right.push(candidate_index);
            } else if candidate_slope > current_slope {
                // this candidate is better.
                self.candidates_right.clear();
                self.candidates_right.push(candidate_index);
                self.best_right = Some(candidate_slope);
            } else {
                /*println!(
                    "Right1:rejecting {} candidate_slope:{}, current_slope:{}",
                    candidate_index, candidate_slope, current_slope
                )*/
            }
        } else {
            // First sample
            self.best_right = Some(candidate_slope);
            self.candidates_right.push(candidate_index);
        }
        /*println!(
            "Right: best slope is {:?} ,best_right:{}",
            self.candidates_right,
            self.best_right.unwrap()
        )*/
    }

    /// clear all data
    fn clear(&mut self) {
        self.best_left = None;
        self.candidates_left.clear();

        self.best_right = None;
        self.candidates_right.clear();
    }
}

/// SiteEvents contains the events happening at a specific point.
/// Line segments have their start and end positions arranged so that line.start.y < line.end.y
/// Sorting is based on their Y-coordinate, secondary the X-coordinate. (line.start.x < line.end.x)
///
/// The 'drop' list contains the line segments that ends in the event point.
/// The 'add' list contains the line segments that starts in the event point.
/// The 'intersection' list contains the line segments that intersects at the event point.
///
pub struct SiteEvent {
    drop: Option<Vec<usize>>,
    add: Option<Vec<usize>>,
    intersection: Option<Vec<usize>>,
}

impl SiteEvent {
    pub(crate) fn with_intersection(i: &[usize]) -> Self {
        Self {
            drop: None,
            add: None,
            intersection: Some(i.to_vec()),
        }
    }

    pub(crate) fn with_drop(l: &[usize]) -> Self {
        Self {
            drop: Some(l.to_vec()),
            add: None,
            intersection: None,
        }
    }

    pub(crate) fn with_add(l: &[usize]) -> Self {
        Self {
            drop: None,
            add: Some(l.to_vec()),
            intersection: None,
        }
    }

    pub fn get_intersections(&self) -> &Option<Vec<usize>> {
        &self.intersection
    }
}

/// Returns *one* point of intersection between the `sweepline` and `other`
/// Second return value is the slope of the line
fn sweepline_intersection<T: GenericVector2>(
    sweepline: T,
    other: &linestring_2d::Line2<T>,
) -> Option<(T::Scalar, T::Scalar)> {
    // line equation: y=slope*x+d => d=y-slope*x => x = (y-d)/slope
    let y1 = other.start.y();
    let y2 = other.end.y();
    let x1 = other.start.x();
    let x2 = other.end.x();
    if ulps_eq!(&y1, &y2) {
        // horizontal line: return to the point right of sweepline.x, if any
        // Any point to the left are supposedly already handled.
        return (sweepline.x() < x2).then_some((x2, T::Scalar::ZERO));
    }

    if ulps_eq!(&x1, &x2) {
        return Some((x1, T::Scalar::INFINITY));
    }

    let slope = (y2 - y1) / (x2 - x1);

    let d = y1 - slope * x1;
    Some(((sweepline.y() - d) / slope, slope))
}

/// Contains the data the sweep-line intersection algorithm needs to operate.
/// Some of these containers are stored inside an Option. This makes it possible
/// to take() them and make the borrow-checker happy.
pub struct IntersectionData<T: GenericVector2> {
    // sweep-line position
    sweepline_pos: T,
    // Stop when first intersection is found
    stop_at_first_intersection: bool,
    // Allow start&end points to intersect
    // i.e. don't report them as an intersections.
    // An endpoint intersecting any other point of another line will still be
    // counted as an intersection.
    pub ignore_end_point_intersections: bool,
    // The unhandled events
    site_events: Option<BTreeMap<SiteEventKey<T>, SiteEvent>>,
    // The input geometry. These lines are re-arranged so that Line.start.y <= Line.end.y
    // These are never changed while the algorithm is running.
    lines: Vec<linestring_2d::Line2<T>>,
}

impl<T: GenericVector2> Default for IntersectionData<T> {
    fn default() -> Self {
        Self {
            sweepline_pos: T::new_2d(-T::Scalar::max_value(), -T::Scalar::max_value()),
            stop_at_first_intersection: false,
            ignore_end_point_intersections: false,
            site_events: Some(BTreeMap::new()),
            lines: Vec::<linestring_2d::Line2<T>>::new(),
        }
    }
}

impl<T: GenericVector2> IntersectionData<T> {
    pub fn get_sweepline_pos(&self) -> &T {
        &self.sweepline_pos
    }

    pub fn get_lines(&self) -> &Vec<linestring_2d::Line2<T>> {
        &self.lines
    }

    pub fn with_stop_at_first_intersection(mut self, value: bool) -> Result<Self, LinestringError> {
        self.stop_at_first_intersection = value;
        Ok(self)
    }

    pub fn with_ignore_end_point_intersections(
        mut self,
        value: bool,
    ) -> Result<Self, LinestringError> {
        self.ignore_end_point_intersections = value;
        Ok(self)
    }

    /// Add data to the input lines.
    /// Sort the end point according to the order of SiteEventKey.
    /// Populate the event queue
    /// TODO: Use IntoIterator instead of Iterator
    pub fn with_lines<IT>(mut self, input_iter: IT) -> Result<Self, LinestringError>
    where
        IT: Iterator<Item = linestring_2d::Line2<T>>,
    {
        let mut site_events = self.site_events.take().ok_or_else(|| {
            LinestringError::InternalError("with_lines() could not take 'site_events'".to_string())
        })?;

        for (index, mut aline) in input_iter.enumerate() {
            if !(aline.start.x().is_finite()
                && aline.start.y().is_finite()
                && aline.end.x().is_finite()
                && aline.end.y().is_finite())
            {
                return Err(LinestringError::InvalidData(
                    "Some of the points are infinite".to_string(),
                ));
            }

            // Re-arrange so that:
            // SiteEvent.pos.start < SiteEvent.pos.end (primary ordering: pos.y, secondary: pos.x)
            if !(SiteEventKey { pos: aline.start }).lt(&(SiteEventKey { pos: aline.end })) {
                std::mem::swap(&mut aline.start, &mut aline.end);
            };

            self.lines.push(aline);

            let key_start = SiteEventKey { pos: aline.start };
            let key_end = SiteEventKey { pos: aline.end };

            // start points goes into the site_event::add list
            if let Some(event) = site_events.get_mut(&key_start) {
                let mut lower = event.add.take().map_or(Vec::<usize>::new(), identity);
                lower.push(index);
                event.add = Some(lower);
            } else {
                let event = SiteEvent::with_add(&[index]);
                let _ = site_events.insert(key_start, event);
            }

            // end points goes into the site_event::drop list
            if let Some(event) = site_events.get_mut(&key_end) {
                let mut upper = event.drop.take().map_or(Vec::<usize>::new(), identity);
                upper.push(index);
                event.drop = Some(upper);
            } else {
                let event = SiteEvent::with_drop(&[index]);
                let _ = site_events.insert(key_end, event);
            }
        }
        self.site_events = Some(site_events);
        #[cfg(feature = "console_trace")]
        self.debug();
        Ok(self)
    }

    #[inline]
    ///
    /// Add a new intersection event to the event queue
    ///
    fn add_intersection_event(
        &self,
        site_events: &mut BTreeMap<SiteEventKey<T>, SiteEvent>,
        position: &SiteEventKey<T>,
        intersecting_lines: &[usize],
    ) {
        if let Some(event) = site_events.get_mut(position) {
            // there were already events at this point
            let mut intersections_added = 0;
            for new_intersection in intersecting_lines.iter() {
                // only add this line as an intersection if the intersection lies
                // at the interior of the line (no end point)
                let i_line = self.lines[*new_intersection];
                if position.pos.is_ulps_eq(
                    i_line.start,
                    T::Scalar::default_epsilon(),
                    T::Scalar::default_max_ulps(),
                ) || position.pos.is_ulps_eq(
                    i_line.end,
                    T::Scalar::default_epsilon(),
                    T::Scalar::default_max_ulps(),
                ) {
                    continue;
                }

                if let Some(prev_intersections) = &mut event.intersection {
                    prev_intersections.push(*new_intersection);
                } else {
                    let new_vec = vec![*new_intersection];
                    event.intersection = Some(new_vec);
                }
                intersections_added += 1;
            }
            if intersections_added > 0 {
                let mut intersections = event.intersection.take().unwrap();
                intersections.sort_unstable();
                intersections.dedup();
                event.intersection = Some(intersections);
            }
        } else {
            // this is the first event at this point
            let event = SiteEvent::with_intersection(intersecting_lines);
            let _ = site_events.insert(*position, event);
        }
    }

    /// Handles input events, returns an iterator containing the results when done.
    #[allow(clippy::type_complexity)]
    pub fn compute(
        mut self,
    ) -> Result<impl ExactSizeIterator<Item = (T, Vec<usize>)>, LinestringError> {
        // make the borrow checker happy by breaking the link between self and all the
        // containers and their iterators.
        let mut active_lines = AHashSet::<usize>::default();
        let mut site_events = self.site_events.take().ok_or_else(|| {
            LinestringError::InternalError("compute() could not take 'site_events'".to_string())
        })?;
        // A list of intersection points and the line segments involved in each intersection
        let mut result: BTreeMap<SiteEventKey<T>, Vec<usize>> = BTreeMap::default();
        // The 'best' lines surrounding the event point but not directly connected to the point.
        let mut neighbour_priority = MinMax::new();

        // The 'best' lines directly connected to the event point.
        let mut connected_priority = MinMaxSlope::new();

        loop {
            if let Some((key, event)) = {
                #[cfg(feature = "map_first_last")]
                {
                    site_events.pop_first()
                }
                #[cfg(not(feature = "map_first_last"))]
                {
                    // emulate pop_first
                    if let Some((first_key, _)) = site_events.iter().next() {
                        let first_key = *first_key;
                        let v = site_events.remove(&first_key).ok_or_else(|| {
                            LinestringError::InternalError(format!(
                                "Found a key to pop but could not remove the value. {}:{}",
                                file!(),
                                line!()
                            ))
                        })?;
                        Some((first_key, v))
                    } else {
                        None
                    }
                }
            } {
                self.handle_event(
                    &key,
                    &event,
                    &mut active_lines,
                    &mut neighbour_priority,
                    &mut connected_priority,
                    &mut site_events,
                    &mut result,
                )?;
                if !result.is_empty() && self.stop_at_first_intersection {
                    break;
                }
            } else {
                self.sweepline_pos = T::new_2d(T::Scalar::max_value(), T::Scalar::max_value());
                break;
            }
        }
        // no need to put the borrowed containers back
        Ok(result.into_iter().map(|x| (x.0.pos, x.1)))
    }

    #[allow(clippy::too_many_arguments)]
    fn handle_event(
        &mut self,
        key: &SiteEventKey<T>,
        event: &SiteEvent,
        active_lines: &mut AHashSet<usize>,
        neighbour_priority: &mut MinMax<T>,
        connected_priority: &mut MinMaxSlope<T>,
        site_events: &mut BTreeMap<SiteEventKey<T>, SiteEvent>,
        result: &mut BTreeMap<SiteEventKey<T>, Vec<usize>>,
    ) -> Result<(), LinestringError> {
        self.sweepline_pos = key.pos;

        let removed_active_lines = event.drop.iter().flatten().count();
        let added_active_lines = event.add.iter().flatten().count();
        let intersections_found = event.intersection.iter().flatten().count();

        #[cfg(feature = "console_trace")]
        {
            println!("*************************************");
            print!(
                "handle_event() sweepline=({:?},{:?})",
                self.sweepline_pos.x(),
                self.sweepline_pos.y(),
            );
            print!(
                ", drop={:?}",
                event.drop.iter().flatten().collect::<Vec<&usize>>()
            );
            print!(
                ", add={:?}",
                event.add.iter().flatten().collect::<Vec<&usize>>()
            );
            print!(
                ", intersection={:?}",
                event.intersection.iter().flatten().collect::<Vec<&usize>>()
            );
            println!(
                ", active:{:?}",
                active_lines.iter().collect::<Vec<&usize>>()
            );
        }

        // Handle points converging at this point:
        // If sum of number of items in 'add' + 'drop' > 1 they must intersect at this point
        if self.ignore_end_point_intersections {
            // only report end point intersections if they collide with
            // a calculated intersection point
            if intersections_found > 0 {
                if removed_active_lines > 0 {
                    self.report_intersections_to_result(
                        result,
                        self.sweepline_pos,
                        event.drop.iter().flatten(),
                    );
                }
                if added_active_lines > 0 {
                    self.report_intersections_to_result(
                        result,
                        self.sweepline_pos,
                        event.add.iter().flatten(),
                    );
                }
                self.report_intersections_to_result(
                    result,
                    self.sweepline_pos,
                    event.intersection.iter().flatten(),
                );
            }
        } else if removed_active_lines + added_active_lines + intersections_found > 1 {
            // report *all* intersections, including the end-to-end intersections
            if removed_active_lines > 0 {
                self.report_intersections_to_result(
                    result,
                    self.sweepline_pos,
                    event.drop.iter().flatten(),
                );
            }
            if added_active_lines > 0 {
                self.report_intersections_to_result(
                    result,
                    self.sweepline_pos,
                    event.add.iter().flatten(),
                );
            }
            if intersections_found > 0 {
                self.report_intersections_to_result(
                    result,
                    self.sweepline_pos,
                    event.intersection.iter().flatten(),
                );
            }
        }

        // remove 'drop' lines from the active lines
        for line_index in event.drop.iter().flatten() {
            let _ = active_lines.remove(line_index);
        }

        //
        // calculate the sweep-line status for this point (aka the 'leftright' struct)
        //
        neighbour_priority.clear();

        'active_lines: for line_index in active_lines.iter() {
            for i in event.intersection.iter().flatten() {
                // lines intersecting at this point can never be left/right candidates
                if i == line_index {
                    continue 'active_lines;
                }
            }

            //print!("(sweepline_intersection id={:?}", line_index);
            if let Some((intersection_x, intersection_slope)) =
                sweepline_intersection(self.sweepline_pos, self.lines.get(*line_index).unwrap())
            {
                //println!(" @{}^{})", intersection_x, intersection_slope);
                neighbour_priority.update(
                    self.sweepline_pos.x(),
                    intersection_x,
                    intersection_slope,
                    *line_index,
                );
            }
        }

        // add the newly found lines
        for l in event.add.iter().flatten() {
            let _ = active_lines.insert(*l);
        }

        if intersections_found + added_active_lines == 0 {
            #[cfg(feature = "console_trace")]
            {
                println!(
                    "neighbours left: {:?}",
                    neighbour_priority.slope.candidates_left
                );
                println!(
                    "neighbours right: {:?}",
                    neighbour_priority.slope.candidates_right
                );
            }
            // this event didn't spawn off new events, check neighbours
            if !neighbour_priority.slope.candidates_left.is_empty()
                && !neighbour_priority.slope.candidates_right.is_empty()
            {
                self.find_new_events(
                    &neighbour_priority.slope.candidates_left,
                    &neighbour_priority.slope.candidates_right,
                    site_events,
                )?;
            }
        } else {
            connected_priority.clear();
            for l in event.add.iter().flatten() {
                connected_priority.update_both(*l, &self.lines);
            }
            for l in event.intersection.iter().flatten() {
                connected_priority.update_both(*l, &self.lines);
            }
            #[cfg(feature = "console_trace")]
            {
                println!(
                    "left connected_priority candidates {:?}",
                    connected_priority.candidates_left
                );
                println!(
                    "right connected_priority candidates {:?}",
                    connected_priority.candidates_right
                );
            }

            if !neighbour_priority.slope.candidates_left.is_empty() {
                #[cfg(feature = "console_trace")]
                println!(
                    "left neighbour_priority candidates {:?}",
                    neighbour_priority.slope.candidates_left
                );

                self.find_new_events(
                    &neighbour_priority.slope.candidates_left,
                    &connected_priority.candidates_left,
                    site_events,
                )?;
            }
            if !neighbour_priority.slope.candidates_right.is_empty() {
                #[cfg(feature = "console_trace")]
                println!(
                    "right neighbour_priority candidates {:?}",
                    neighbour_priority.slope.candidates_right
                );

                self.find_new_events(
                    &neighbour_priority.slope.candidates_right,
                    &connected_priority.candidates_right,
                    site_events,
                )?;
            }
        }
        #[cfg(feature = "console_trace")]
        {
            println!("Post active lines: {:?}", active_lines);
            println!();
        }
        Ok(())
    }

    fn find_new_events(
        &mut self,
        left: &[usize],
        right: &[usize],
        site_events: &mut BTreeMap<SiteEventKey<T>, SiteEvent>,
    ) -> Result<(), LinestringError> {
        for left_i in left.iter() {
            for right_i in right.iter() {
                let left_l = self.lines[*left_i];
                let right_l = self.lines[*right_i];
                if left_l.end.is_ulps_eq(
                    right_l.end,
                    T::Scalar::default_epsilon(),
                    T::Scalar::default_max_ulps(),
                ) {
                    // if endpoints are equal they will already be in the event queue
                    continue;
                }
                #[cfg(feature = "console_trace")]
                print!("testing intersection between {} and {}: ", left_i, right_i);

                if let Some(intersection_p) = left_l.intersection_point(right_l) {
                    let intersection_p = intersection_p.single();
                    if !intersection_p.x().is_finite() || !intersection_p.y().is_finite() {
                        return Err(LinestringError::InvalidData(format!(
                            "Input data has intersection at invalid position x:{:?}, y:{:?}",
                            intersection_p.x(),
                            intersection_p.y()
                        )));
                    }
                    // don't allow intersection 'behind' or 'at' current sweep-line position
                    if intersection_p.y() >= self.sweepline_pos.y()
                        && !(intersection_p.y() == self.sweepline_pos.y()
                            && intersection_p.x() < self.sweepline_pos.x())
                        && !intersection_p.is_ulps_eq(
                            self.sweepline_pos,
                            T::Scalar::default_epsilon(),
                            T::Scalar::default_max_ulps(),
                        )
                    {
                        #[cfg(feature = "console_trace")]
                        println!(
                            "Lines {:?} and {:?} intersects at {:?}",
                            left_i, right_i, intersection_p
                        );

                        self.add_intersection_event(
                            site_events,
                            &SiteEventKey {
                                pos: intersection_p,
                            },
                            &[*left_i, *right_i],
                        )
                    }
                } else {
                    #[cfg(feature = "console_trace")]
                    println!(" no intersection");
                }
            }
        }
        Ok(())
    }

    fn report_intersections_to_result<'a, I: Iterator<Item = &'a usize>>(
        &mut self,
        result: &mut BTreeMap<SiteEventKey<T>, Vec<usize>>,
        pos: T,
        intersecting_lines: I,
    ) {
        let key = SiteEventKey { pos };

        let value = if let Some(value) = result.get_mut(&key) {
            value
        } else {
            let _ = result.insert(key, Vec::default());
            result.get_mut(&key).unwrap()
        };

        for line in intersecting_lines {
            value.push(*line);
            #[cfg(feature = "console_trace")]
            println!("Reported an intersection {:?} for line #{}", pos, line);
        }
        value.sort_unstable();
    }

    #[cfg(feature = "console_trace")]
    fn debug(&self) {
        println!("Stored item in this order:");
        for k in self.site_events.as_ref().unwrap().iter().map(|kv| *kv.0) {
            print!("{:?}, ", k);
        }
        println!();
    }
}
