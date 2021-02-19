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
use fltk::valuator::HorNiceSlider;
use fltk::{app, button::*, draw::*, frame::*, window::*};
use linestring::cgmath_2d::LineString2;
use linestring::LinestringError;
use std::cell::RefCell;
use std::rc::Rc;

// frame size
const HF: i32 = 590;
const WF: i32 = 790;

// window size
const H: i32 = 650;
const W: i32 = 800;

struct SharedData {
    //distance: f32,
    lines: Vec<LineString2<f32>>,
}

fn main() -> Result<(), LinestringError> {
    let app = app::App::default().with_scheme(app::Scheme::Gtk);

    let mut wind = Window::new(100, 100, W, H, "Ramer–Douglas–Peucker 2d Demo");
    let mut frame = Frame::new(5, 5, WF, HF, "");
    frame.set_color(Color::White);
    frame.set_frame(FrameType::DownBox);

    let mut slider = HorNiceSlider::new(5, 5 + HF, WF, 25, "Move the slider to draw!!!");
    slider.set_value(0.0);
    slider.set_frame(FrameType::PlasticUpBox);

    wind.end();
    wind.show();

    let shared_data_rc = Rc::from(RefCell::from(SharedData {
        //distance: 0.0,
        lines: Vec::new(),
    }));

    let offs = Offscreen::new(WF, HF).unwrap();
    #[cfg(not(target_os = "macos"))]
    {
        offs.begin();
        set_draw_color(Color::White);
        draw_rectf(0, 0, WF, HF);
        offs.end();
    }

    let offs = Rc::from(RefCell::from(offs));
    let offs_rc = offs.clone();

    let slider_shared_data = Rc::clone(&shared_data_rc);
    slider.set_callback2(move |b| {
        let shared_data = slider_shared_data.borrow_mut();

        let distance = b.value() as f32 * 200.0;
        b.set_label(
            ("           Distance:".to_string()
                + &distance.to_string()
                + (&"           ".to_string()))
                .as_str(),
        );
        offs_rc.borrow_mut().begin();
        set_draw_color(Color::White);
        draw_rectf(0, 0, WF, HF);
        set_line_style(LineStyle::Solid, 1);

        for l in shared_data.lines.iter() {
            set_draw_color(Color::Green);
            for a_line in l.as_lines() {
                draw_line(
                    a_line.start.x as i32,
                    a_line.start.y as i32,
                    a_line.end.x as i32,
                    a_line.end.y as i32,
                );
            }

            set_draw_color(Color::Black);
            let sl = l.simplify(distance);
            for a_line in sl.as_lines() {
                draw_line(
                    a_line.start.x as i32,
                    a_line.start.y as i32,
                    a_line.end.x as i32,
                    a_line.end.y as i32,
                );
            }
        }
        redraw();
        offs_rc.borrow_mut().end();
    });

    add_data(shared_data_rc.clone())?;

    let offs_rc = offs.clone();

    frame.draw(move || {
        if offs_rc.borrow().is_valid() {
            offs_rc.borrow().copy(5, 5, WF, HF, 0, 0);
        } else {
            offs_rc.borrow_mut().begin();
            set_draw_color(Color::White);
            draw_rectf(0, 0, WF, HF);
            offs_rc.borrow_mut().end();
        }
    });

    app.run().unwrap();
    Ok(())
}

fn add_data(data: Rc<RefCell<SharedData>>) -> Result<(), LinestringError> {
    let mut data_b = data.borrow_mut();
    data_b.lines.clear();

    // Add y=e^-x*cos(2πx)
    let mut line: Vec<cgmath::Point2<f32>> = Vec::new();
    for x in (0..150).skip(1) {
        let x = x as f32 / 20.0;
        let y: f32 = std::f32::consts::E.powf(-x) * (x * 2.0 * std::f32::consts::PI).cos();
        line.push(cgmath::Point2::new(50.0 + x * 75.0, 200.0 + y * 300.0));
    }
    let line = LineString2::<f32>::default()
        .with_points(line)
        .with_connected(false);
    data_b.lines.push(line);

    // Add a wobbly circle
    let mut line: Vec<cgmath::Point2<f32>> = Vec::new();
    for angle in (0..358).skip(2) {
        let x: f32 = 400.0
            + (angle as f32).to_radians().cos() * 250.0
            + 5.0 * (angle as f32 * 8.523).to_radians().sin();
        let y: f32 = 400.0
            + (angle as f32).to_radians().sin() * 150.0
            + 5.0 * (angle as f32 * 2.534).to_radians().cos();
        line.push(cgmath::Point2::new(x, y));
    }
    let line = LineString2::<f32>::default()
        .with_points(line)
        .with_connected(true);

    data_b.lines.push(line);

    Ok(())
}
