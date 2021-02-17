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

#[cfg(feature = "impl-cgmath")]
use linestring::cgmath_2d::LineString2;
use linestring::LinestringError;

use fltk::*;
use std::cell::RefCell;
use std::rc::Rc;

const WINDOW_SIZE: i32 = 800;

struct SharedData {
    distance: f32,
    lines: Vec<LineString2<f32>>,
    _window_center: (i32, i32),
}

#[cfg(feature = "impl-cgmath")]
fn main() -> Result<(), LinestringError> {
    let app = app::App::default();
    let mut wind = window::Window::default()
        .with_size(WINDOW_SIZE, WINDOW_SIZE)
        .center_screen()
        .with_label("linestring.simplify() demo");

    wind.set_color(Color::Black);
    wind.end();
    wind.show();
    let shared_data_rc = Rc::from(RefCell::from(SharedData {
        distance: 1.0,
        lines: Vec::new(),
        _window_center: (WINDOW_SIZE / 2, WINDOW_SIZE / 2),
    }));

    add_data(shared_data_rc.clone())?;

    let shared_data_clone = shared_data_rc.clone();
    // This is called whenever the window is drawn and redrawn (in the event loop)
    wind.draw(move || {
        let alg_data_b = shared_data_clone.borrow();

        for l in alg_data_b.lines.iter() {
            draw::set_draw_color(Color::Green);
            for a_line in l.as_lines() {
                draw::draw_line(
                    a_line.start.x as i32,
                    a_line.start.y as i32,
                    a_line.end.x as i32,
                    a_line.end.y as i32,
                );
            }

            draw::set_draw_color(Color::White);
            let sl = l.simplify(alg_data_b.distance);
            for a_line in sl.as_lines() {
                draw::draw_line(
                    a_line.start.x as i32,
                    a_line.start.y as i32,
                    a_line.end.x as i32,
                    a_line.end.y as i32,
                );
            }
        }
    });
    wind.handle(move |ev| match ev {
        enums::Event::Released => {
            let event = &app::event_coords();
            println!("mouse at {:?}", event);
            true
        }
        _ => false,
    });

    while app.wait() {
        wind.redraw();
        if !cfg!(windows) {
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }
    Ok(())
}

fn add_data(data: Rc<RefCell<SharedData>>) -> Result<(), LinestringError> {
    let mut data_b = data.borrow_mut();
    data_b.lines.clear();

    let points = vec![
        [77f32, 613.],
        [689., 650.],
        [710., 467.],
        [220., 200.],
        [120., 300.],
        [100., 100.],
    ];
    let mut line: LineString2<f32> = points.into_iter().collect();
    line.connected = true;
    data_b.lines.push(line);

    let mut line: Vec<cgmath::Point2<f32>> = Vec::new();
    for angle in (0..358).skip(2) {
        let x: f32 = 400.0 + (angle as f32).to_radians().cos() * 150.0;
        let y: f32 = 400.0 + (angle as f32).to_radians().sin() * 100.0;
        line.push(cgmath::Point2::new(x, y));
    }
    let mut line: LineString2<f32> = line.into_iter().collect();
    line.connected = true;

    data_b.lines.push(line);

    data_b.distance = 10.0;
    Ok(())
}
