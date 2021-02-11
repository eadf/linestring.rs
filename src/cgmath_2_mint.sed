# Simple sed scripts converting from cgmath_impl.rs to mint_impl.rs
# This could probably be done as a Rust macro but I doubt it would be as easy as 4 lines of sed script

# Use this shell command to generate mint_impl.rs:
# sed -f cgmath_2_mint.sed < cgmath_impl.rs > mint_impl.rs

s/cgmath::Point3/mint::Point3/g
s/cgmath::Point2/mint::Point2/g
s/Original file, edit this one and convert to the others/Auto generated file, do NOT edit/g
