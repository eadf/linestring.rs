# simple sed scripts converting from cgmath_impl.rs to nalgebra_impl.rs

s/T: Copy +/T: nalgebra::Scalar + Copy +/g
s/cgmath::Point3/nalgebra::Point3/g
s/cgmath::Point2/nalgebra::Point2/g
s/Original file, edit this one and convert to the others/Auto generated file, do NOT edit/g
