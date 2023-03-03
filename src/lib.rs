pub(crate) const EPSILON: f32 = 1e-7;

#[macro_use]
mod linked_list;
mod mesher;
mod triangulator;
mod vec2;

pub use triangulator::{Triangulator, TriangulatorError};

#[cfg(test)]
mod test;
