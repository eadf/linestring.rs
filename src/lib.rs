#![deny(non_camel_case_types)]
#![deny(unused_parens)]
#![deny(non_upper_case_globals)]
#![deny(unused_qualifications)]
#![deny(unused_results)]
#![deny(unused_imports)]

#[cfg(feature = "impl-nalgebra")]
pub mod nalgebra_impl;

#[cfg(feature = "impl-cgmath")]
pub mod cgmath_impl;

#[cfg(feature = "impl-mint")]
pub mod mint_impl;

#[cfg(feature = "impl-vec")]
pub mod vec_impl;
