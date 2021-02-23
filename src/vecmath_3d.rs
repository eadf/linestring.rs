/*
Line and line segment library for 2d and 3d.

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

Linestring Copyright (C) 2021 eadf

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

#[inline(always)]
#[cfg(feature = "impl-vecmath")]
/// vec_math seems to lack the affine-transform-of-3d-point (matrix4x4) function i need.
/// In order to make vec_math work the same way as cgmath and nalgebra i had to do this little
/// hack.
/// Todo: vecmath does not use & references to stack-objects, not even for objects as large as a
/// 4x4 f64 matrix. nalgebra does. I wonder what's the best choice..
pub fn col_mat4_transform_pos3<T>(matrix4x4: &vecmath::Matrix4<T>, pos: [T; 3]) -> [T; 3]
where
    T: num_traits::Float + std::fmt::Debug + approx::AbsDiffEq + approx::UlpsEq,
{
    // homogeneous point
    let x = pos[0];
    let y = pos[1];
    let z = pos[2];
    let w = T::one();
    [
        matrix4x4[0][0] * x + matrix4x4[1][0] * y + matrix4x4[2][0] * z + matrix4x4[3][0] * w,
        matrix4x4[0][1] * x + matrix4x4[1][1] * y + matrix4x4[2][1] * z + matrix4x4[3][1] * w,
        matrix4x4[0][2] * x + matrix4x4[1][2] * y + matrix4x4[2][2] * z + matrix4x4[3][2] * w,
    ]
}
