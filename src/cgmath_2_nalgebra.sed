# Simple sed scripts converting from cgmath_impl.rs to nalgebra_impl.rs.
# This should probably be done by a rust macro but for now this simple sed script will do


# Use this shell command to generate nalgebra_impl.rs:
# sed -f cgmath_2_nalgebra.sed < cgmath_impl.rs > nalgebra_impl.rs

s/cgmath::BaseFloat/nalgebra::Scalar + nalgebra::RealField/g
s/cgmath::Point3/nalgebra::Point3/g
s/cgmath::Point2/nalgebra::Point2/g
s/cgmath::Matrix4/nalgebra::Matrix4/g
s/cgmath::Matrix3/nalgebra::Matrix3/g
s/cgmath::Transform2/nalgebra::Transform2/g
s/cgmath::Transform3/nalgebra::Transform3/g
s/mat.transform_point(\*/mat.transform_point(/g
s/use cgmath::Transform;//g
s/mat\.transform_point(aabb_min_max/mat\.transform_point(\&aabb_min_max/g
s/Original file, edit this one and convert to the others/Auto generated file, do NOT edit/g
