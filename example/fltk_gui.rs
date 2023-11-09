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

use fltk::{
    app,
    app::{redraw, MouseWheel},
    draw::*,
    enums::*,
    frame::*,
    prelude::*,
    valuator::HorNiceSlider,
    window::*,
};
use linestring::linestring_2d::{Aabb2, LineString2, SimpleAffine};
use std::{cell::RefCell, rc::Rc};
use vector_traits::glam;

// frame size
const HF: i32 = 565;
const WF: i32 = 790;

// Frame offset (border size)
const FO: i32 = 5;

// window size
const H: i32 = 650;
const W: i32 = 800;

// Slider height
const SH: i32 = 25;

type T = f32;

#[derive(Debug, Clone, Copy)]
pub enum GuiMessage {
    SliderRdpChanged(f32),
    SliderVwChanged(usize),
}

struct SharedData {
    lines: Vec<LineString2<glam::Vec2>>,
    last_message: Option<GuiMessage>,
    affine: SimpleAffine<glam::Vec2>,
}

fn main() {
    let app = app::App::default().with_scheme(app::Scheme::Gtk);

    let mut wind = Window::new(
        100,
        100,
        W,
        H + SH * 1,
        "Ramer–Douglas–Peucker & Visvalingam–Whyatt simplification + Self intersection test",
    );
    let mut frame = Frame::new(FO, FO, WF, HF + SH * 1, "");
    frame.set_color(Color::White);
    frame.set_frame(FrameType::DownBox);

    let mut slider_d =
        HorNiceSlider::new(FO, FO + HF, WF, SH, "Ramer–Douglas–Peucker distance:0.0");
    slider_d.set_value(0.0);
    slider_d.set_color(Color::Green);
    slider_d.set_frame(FrameType::PlasticUpBox);

    let mut slider_n = HorNiceSlider::new(
        FO,
        FO + HF + SH * 2,
        WF,
        SH,
        "Visvalingam–Whyatt deleted points:0",
    );
    slider_n.set_value(0.0);
    slider_n.set_color(Color::Blue);
    slider_n.set_frame(FrameType::PlasticUpBox);

    wind.end();
    wind.show();

    let shared_data_rc = Rc::from(RefCell::from(SharedData {
        lines: Vec::new(),
        last_message: None,
        // default is a one-to-one mapping
        affine: SimpleAffine::default(),
    }));
    {
        // Flip the Y coordinate so that the geometry is Quadrant 0, while screen is Quadrant 4
        let mut shared_data_bm = shared_data_rc.borrow_mut();
        let aabb =
            Aabb2::<glam::Vec2>::with_points(&[(0.0, 0.0).into(), (W as f32, H as f32).into()]);
        shared_data_bm.affine = SimpleAffine::new(&aabb, &aabb).unwrap();
        shared_data_bm.affine.scale[1] *= -1.0;
    }
    #[cfg(not(target_os = "macos"))]
    {
        set_draw_color(Color::White);
        draw_rectf(0, 0, WF, HF);
    }

    let (sender, receiver) = app::channel::<GuiMessage>();
    sender.send(GuiMessage::SliderRdpChanged(0.0));

    slider_d.set_callback(move |s| {
        let distance = s.value() as T * 400.0;
        s.set_label(
            ("           Ramer–Douglas–Peucker distance:".to_string()
                + &distance.to_string()
                + (&"           ".to_string()))
                .as_str(),
        );
        sender.send(GuiMessage::SliderRdpChanged(distance));
    });

    slider_n.set_callback(move |s| {
        let number_to_remove = (s.value() as T * 200.0) as usize;
        s.set_color(Color::Blue);
        s.set_label(
            ("           Visvalingam–Whyatt deleted points:".to_string()
                + &number_to_remove.to_string()
                + (&"           ".to_string()))
                .as_str(),
        );
        sender.send(GuiMessage::SliderVwChanged(number_to_remove));
    });

    add_data(Rc::clone(&shared_data_rc));

    let shared_data_c = Rc::clone(&shared_data_rc);
    frame.draw(move |_| {
        let shared_data_b = shared_data_c.borrow();
        let make_line = |line: [T; 4], cross: bool| {
            let line = shared_data_b.affine.transform_ab_a(line);
            if let Ok(line) = line {
                draw_line(
                    line[0] as i32,
                    line[1] as i32,
                    line[2] as i32,
                    line[3] as i32,
                );
                if cross {
                    draw_line(
                        line[0] as i32 - 2,
                        line[1] as i32 - 2,
                        line[0] as i32 + 2,
                        line[1] as i32 + 2,
                    );
                    draw_line(
                        line[2] as i32 + 2,
                        line[3] as i32 - 2,
                        line[2] as i32 - 2,
                        line[3] as i32 + 2,
                    );
                }
            }
        };

        match shared_data_b.last_message {
            Some(GuiMessage::SliderRdpChanged(distance)) => {
                set_draw_color(Color::White);
                draw_rectf(6, 6, WF - 6, HF - 6);
                set_line_style(LineStyle::Solid, 1);

                for l in shared_data_b.lines.iter() {
                    if l.is_self_intersecting().unwrap() {
                        set_draw_color(Color::Red);
                    } else {
                        set_draw_color(Color::Black);
                    }

                    for a_line in l.line_iter() {
                        make_line(
                            [a_line.start.x, a_line.start.y, a_line.end.x, a_line.end.y],
                            false,
                        );
                    }

                    let simplified_line = l.simplify_rdp(distance);
                    if simplified_line.is_self_intersecting().unwrap() {
                        set_draw_color(Color::Red);
                    } else {
                        set_draw_color(Color::Green);
                    }
                    for a_line in simplified_line.line_iter() {
                        make_line(
                            [a_line.start.x, a_line.start.y, a_line.end.x, a_line.end.y],
                            true,
                        );
                    }
                    /*
                    set_draw_color(Color::Green);
                    let hull = linestring::linestring_2d::convex_hull::ConvexHull::<f32>::graham_scan(
                        &simplified_line,
                    );
                    for a_line in hull.as_lines_iter() {
                        make_line(
                            [a_line.start.x, a_line.start.y, a_line.end.x, a_line.end.y],
                            false,
                        );
                    }
                    */
                }
            }
            Some(GuiMessage::SliderVwChanged(number_to_remove)) => {
                set_draw_color(Color::White);
                draw_rectf(6, 6, WF - 6, HF - 6);
                set_line_style(LineStyle::Solid, 1);

                for l in shared_data_b.lines.iter() {
                    if l.is_self_intersecting().unwrap() {
                        set_draw_color(Color::Red);
                    } else {
                        set_draw_color(Color::Black);
                    }

                    for a_line in l.line_iter() {
                        make_line(
                            [a_line.start.x, a_line.start.y, a_line.end.x, a_line.end.y],
                            false,
                        );
                    }

                    let simplified_line = l.simplify_vw(number_to_remove);
                    if simplified_line.is_self_intersecting().unwrap() {
                        set_draw_color(Color::Red);
                    } else {
                        set_draw_color(Color::Blue);
                    }
                    for a_line in simplified_line.line_iter() {
                        make_line(
                            [a_line.start.x, a_line.start.y, a_line.end.x, a_line.end.y],
                            true,
                        );
                    }
                    /*
                    set_draw_color(Color::Green);
                    let hull = linestring::linestring_2d::convex_hull::ConvexHull::<f32>::graham_scan(
                        &simplified_line,
                    );
                    for a_line in hull.as_lines_iter() {
                        make_line(
                            [a_line.start.x, a_line.start.y, a_line.end.x, a_line.end.y],
                            false,
                        );
                    }*/
                    //let _ = linestring::linestring_2d::convex_hull::GrahamScan::<f32>::scan(
                    //   &simplified_line);
                }
            }
            None => (),
        }
    });

    let shared_data_c = Rc::clone(&shared_data_rc);
    // mouse_drag is only used inside this closure, so it does not need to be placed in
    // shared_data
    let mut mouse_drag: Option<(i32, i32)> = None;
    wind.handle(move |_, ev| match ev {
        fltk::enums::Event::MouseWheel => {
            let event = &app::event_coords();
            let mut shared_data_bm = shared_data_c.borrow_mut();
            let event_dy = match app::event_dy() {
                // arbitrary selected value
                MouseWheel::Up => 3,
                MouseWheel::Down => -3,
                _ => 0,
            };
            let reverse_middle = shared_data_bm
                .affine
                .transform_ba(glam::Vec2::from([event.0 as T, event.1 as T]));
            if reverse_middle.is_err() {
                println!("{:?}", reverse_middle.err().unwrap());
                return false;
            }
            let reverse_middle = reverse_middle.unwrap();
            if event_dy != 0 {
                let scale_mod = 1.01_f32.powf(event_dy as T);
                shared_data_bm.affine.scale[0] *= scale_mod;
                shared_data_bm.affine.scale[1] *= scale_mod;
            }
            let new_middle = shared_data_bm.affine.transform_ab(glam::Vec2::from([
                reverse_middle[0] as T,
                reverse_middle[1] as T,
            ]));
            if new_middle.is_err() {
                println!("{:?}", new_middle.err().unwrap());
                return false;
            }
            let new_middle = new_middle.unwrap();
            // When zooming we want the shape at the mouse position to remain
            // at the same relative position.
            shared_data_bm.affine.b_offset[0] += (event.0 as T) - new_middle[0];
            shared_data_bm.affine.b_offset[1] += (event.1 as T) - new_middle[1];

            //println!("mouse wheel at dy:{:?} scale:{:?}", event_dy, shared_data_bm.visualizer.affine.scale);
            redraw();
            true
        }
        fltk::enums::Event::Drag => {
            let event = &app::event_coords();
            if mouse_drag.is_none() {
                mouse_drag = Some(*event);
            } else {
                let md = mouse_drag.unwrap();
                let mut shared_data_bm = shared_data_c.borrow_mut();
                shared_data_bm.affine.b_offset[0] += (event.0 - md.0) as T;
                shared_data_bm.affine.b_offset[1] += (event.1 - md.1) as T;
                mouse_drag = Some(*event);
                redraw();
            }
            true
        }
        fltk::enums::Event::Released => {
            if mouse_drag.is_some() {
                mouse_drag = None;
            }
            true
        }
        _ => false,
    });

    let shared_data_c = Rc::clone(&shared_data_rc);
    while app.wait() {
        if let Some(msg) = receiver.recv() {
            let mut shared_data_bm = shared_data_c.borrow_mut();
            shared_data_bm.last_message = Some(msg);
            redraw();
        }
    }
}

// add some test data
fn add_data(data: Rc<RefCell<SharedData>>) {
    let mut data_b = data.borrow_mut();
    data_b.lines.clear();

    // Add y=e^-x*cos(2πx)
    let mut line: Vec<glam::Vec2> = Vec::with_capacity(150);
    for x in (0..150).skip(1) {
        let x = (x as T) / 20.0;
        let y: f32 = std::f32::consts::E.powf(-x) * (x * 2.0 * std::f32::consts::PI).cos();
        line.push(glam::Vec2::new(50.0 + x * 75.0, 200.0 + y * 300.0));
    }
    let line = LineString2::with_vec(line);
    data_b.lines.push(line);

    // Add a wobbly circle
    let mut line: Vec<glam::Vec2> = Vec::with_capacity(360);
    for angle in (0..358).step_by(2) {
        let x: f32 = 400.0
            + (angle as T).to_radians().cos() * 250.0
            + (FO as T) * (angle as T * 8.523).to_radians().sin();
        let y: f32 = 400.0
            + (angle as T).to_radians().sin() * 150.0
            + (FO as T) * (angle as T * 2.534).to_radians().cos();
        line.push(glam::Vec2::new(x, y));
    }
    line.push(line[0]);
    // Add an extra point that will cause self-intersection when simplified too much
    line.push(glam::Vec2::new(250_f32, 300.0));

    let line = LineString2::with_vec(line);

    data_b.lines.push(line);
}
