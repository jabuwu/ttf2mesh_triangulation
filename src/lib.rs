#[macro_use]
mod linked_list;
mod triangulation;
mod vec2;

pub use triangulation::{Triangulator, TriangulatorError};

#[cfg(test)]
mod test;
