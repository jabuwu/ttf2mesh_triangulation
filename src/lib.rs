#[macro_use]
mod linked_list;
mod triangulation;
mod vec2;

pub use {
    triangulation::{Triangulator, TriangulatorError},
    vec2::Vec2,
};

#[cfg(test)]
mod test;
