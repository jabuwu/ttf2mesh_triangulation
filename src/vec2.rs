use std::ops::{Add, Index, IndexMut, Mul, Sub};

#[repr(C)]
#[derive(Default, Clone, Copy, Debug, PartialEq)]
pub(crate) struct Vec2 {
    pub(crate) x: f32,
    pub(crate) y: f32,
}

impl Vec2 {
    pub(crate) const ZERO: Vec2 = Vec2::new(0., 0.);

    pub(crate) const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    pub(crate) fn length_squared(&self) -> f32 {
        self.dot(*self)
    }

    pub(crate) fn length(&self) -> f32 {
        self.dot(*self).sqrt()
    }

    pub(crate) fn dot(&self, rhs: Vec2) -> f32 {
        (self.x * rhs.x) + (self.y * rhs.y)
    }

    pub(crate) fn cross(&self, rhs: Vec2) -> f32 {
        (self.x * rhs.y) - (self.y * rhs.x)
    }

    pub(crate) fn proj(&self, e1: Vec2, e2: Vec2) -> Vec2 {
        Vec2::new(self.dot(e1), self.dot(e2))
    }
}

impl Index<usize> for Vec2 {
    type Output = f32;

    fn index(&self, index: usize) -> &Self::Output {
        match index {
            0 => &self.x,
            1 => &self.y,
            _ => panic!("index out of bounds"),
        }
    }
}

impl IndexMut<usize> for Vec2 {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            _ => panic!("index out of bounds"),
        }
    }
}

impl Add<Vec2> for Vec2 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Self {
            x: self.x.add(rhs.x),
            y: self.y.add(rhs.y),
        }
    }
}

impl Sub<Vec2> for Vec2 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        Self {
            x: self.x.sub(rhs.x),
            y: self.y.sub(rhs.y),
        }
    }
}

impl Mul<f32> for Vec2 {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self {
        Self {
            x: self.x.mul(rhs),
            y: self.y.mul(rhs),
        }
    }
}
