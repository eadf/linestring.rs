/*
Linestring library.

Copyright (C) 2021 eadf https://github.com/eadf

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.

Also add information on how to contact you by electronic and paper mail.

If the program does terminal interaction, make it output a short notice like
this when it starts in an interactive mode:

intersection2d Copyright (C) 2021 eadf

This program comes with ABSOLUTELY NO WARRANTY; for details type `show w'.

This is free software, and you are welcome to redistribute it under certain
conditions; type `show c' for details.

The hypothetical commands `show w' and `show c' should show the appropriate
parts of the General Public License. Of course, your program's commands might
be different; for a GUI interface, you would use an "about box".

You should also get your employer (if you work as a programmer) or school,
if any, to sign a "copyright disclaimer" for the program, if necessary. For
more information on this, and how to apply and follow the GNU GPL, see <https://www.gnu.org/licenses/>.

The GNU General Public License does not permit incorporating your program
into proprietary programs. If your program is a subroutine library, you may
consider it more useful to permit linking proprietary applications with the
library. If this is what you want to do, use the GNU Lesser General Public
License instead of this License. But first, please read <https://www.gnu.org/
licenses /why-not-lgpl.html>.
*/
use fltk::app::redraw;
use fltk::app::MouseWheel;
use fltk::valuator::HorNiceSlider;
use fltk::{app, draw::*, frame::*, window::*};
use fltk::{enums::*, prelude::*};
use linestring::cgmath_2d::{Aabb2, LineString2, SimpleAffine};
use std::cell::RefCell;
use std::rc::Rc;

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
    lines: Vec<LineString2<f32>>,
    last_message: Option<GuiMessage>,
    affine: SimpleAffine<T>,
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
        let aabb = Aabb2::from([0.0, 0.0, W as f32, H as f32]);
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

                    for a_line in l.as_lines_iter() {
                        make_line(
                            [a_line.start.x, a_line.start.y, a_line.end.x, a_line.end.y],
                            false,
                        );
                    }

                    let simplified_line = l.simplify(distance);
                    if simplified_line.is_self_intersecting().unwrap() {
                        set_draw_color(Color::Red);
                    } else {
                        set_draw_color(Color::Green);
                    }
                    for a_line in simplified_line.as_lines_iter() {
                        make_line(
                            [a_line.start.x, a_line.start.y, a_line.end.x, a_line.end.y],
                            true,
                        );
                    }
                    /*
                    set_draw_color(Color::Green);
                    let hull = linestring::cgmath_2d::convex_hull::ConvexHull::<f32>::graham_scan(
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

                    for a_line in l.as_lines_iter() {
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
                    for a_line in simplified_line.as_lines_iter() {
                        make_line(
                            [a_line.start.x, a_line.start.y, a_line.end.x, a_line.end.y],
                            true,
                        );
                    }
                    /*
                    set_draw_color(Color::Green);
                    let hull = linestring::cgmath_2d::convex_hull::ConvexHull::<f32>::graham_scan(
                        &simplified_line,
                    );
                    for a_line in hull.as_lines_iter() {
                        make_line(
                            [a_line.start.x, a_line.start.y, a_line.end.x, a_line.end.y],
                            false,
                        );
                    }*/
                    //let _ = linestring::cgmath_2d::convex_hull::GrahamScan::<f32>::scan(
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
                .transform_ba(&cgmath::Point2::from([event.0 as T, event.1 as T]));
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
            let new_middle = shared_data_bm.affine.transform_ab(&cgmath::Point2::from([
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

fn add_data(data: Rc<RefCell<SharedData>>) {
    let mut data_b = data.borrow_mut();
    data_b.lines.clear();

    // Add y=e^-x*cos(2πx)
    let mut line: Vec<cgmath::Point2<f32>> = Vec::new();
    for x in (0..150).skip(1) {
        let x = x as T / 20.0;
        let y: f32 = std::f32::consts::E.powf(-x) * (x * 2.0 * std::f32::consts::PI).cos();
        line.push(cgmath::Point2::new(50.0 + x * 75.0, 200.0 + y * 300.0));
    }
    let line = LineString2::<f32>::default()
        .with_points(line)
        .with_connected(false);
    data_b.lines.push(line);

    // Add a wobbly circle
    let mut line: Vec<cgmath::Point2<f32>> = Vec::with_capacity(360);
    for angle in (0..358).step_by(2) {
        let x: f32 = 400.0
            + (angle as T).to_radians().cos() * 250.0
            + (FO as T) * (angle as T * 8.523).to_radians().sin();
        let y: f32 = 400.0
            + (angle as T).to_radians().sin() * 150.0
            + (FO as T) * (angle as T * 2.534).to_radians().cos();
        line.push(cgmath::Point2::new(x, y));
    }
    // Add an extra point that will cause self-intersection when simplified too much
    line.push(cgmath::Point2::new(250_f32, 300.0));

    let line = LineString2::<f32>::default()
        .with_points(line)
        .with_connected(true);

    data_b.lines.push(line);
}
