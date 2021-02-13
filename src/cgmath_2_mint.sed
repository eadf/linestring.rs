# Simple sed scripts converting from cgmath_impl.rs to mint_impl.rs
# This should probably be done by a rust macro but for now this simple sed script will do


# Use this shell command to generate mint_impl.rs:
# sed -f cgmath_2_mint.sed < cgmath_impl.rs > mint_impl.rs

s/cgmath::Point3/mint::Point3/g
s/cgmath::Point2/mint::Point2/g
s/+ cgmath::BaseFloat//g
s/mint::Point3::new(/mint::Point3::from([/g
s/cgmath::Matrix3/mint::ColumnMatrix3/g
s/cgmath::Matrix4/mint::ColumnMatrix4/g
s/use cgmath::Transform;//g

# mint does not implement anything so some features will be disabled. e.g. transform()
s!\/\/\#\[cfg(not(feature="impl-mint"))\]!\#\[cfg(not(feature="impl-mint"))\]!g

s/Original file, edit this one and convert to the others/Auto generated file, do NOT edit/g
