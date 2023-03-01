#[macro_use]
mod linked_list;

mod triangulation;
pub use triangulation::{Triangulator, TriangulatorError};

#[cfg(test)]
mod test;
