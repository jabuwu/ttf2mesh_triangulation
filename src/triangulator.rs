use std::mem::transmute;

use snafu::Snafu;

use crate::{mesher::Mesher, vec2::Vec2, EPSILON};

#[derive(Default)]
pub struct Triangulator {
    pub(crate) contours: Vec<Contour>,
    pub(crate) total_points: usize,
}

impl Triangulator {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_contour(
        &mut self,
        identifier: usize,
        points: Vec<[f32; 2]>,
    ) -> Result<(), TriangulatorError> {
        if points.len() < 3 {
            return Err(TriangulatorError::Incomplete);
        }

        self.total_points += points.len();
        self.contours.push(Contour {
            identifier,
            points: unsafe { transmute(points) },
            is_hole: false,
            nested_to: None,
        });

        for contour_index in 0..self.contours.len() {
            let mut nested_to = None;
            let is_hole = !ttf_outline_contour_info_majority(
                self,
                self.contours[contour_index].identifier,
                contour_index,
                &mut nested_to,
            );
            self.contours[contour_index].nested_to = nested_to;
            self.contours[contour_index].is_hole = is_hole;
        }

        Ok(())
    }

    pub fn triangulate(&self) -> Result<Vec<[[f32; 2]; 3]>, TriangulatorError> {
        if !self.contours.is_empty() {
            let mut mesher = Mesher::new(self);
            mesher.process(128, true, true)?;
            Ok(unsafe { transmute(mesher.triangles()) })
        } else {
            Err(TriangulatorError::Incomplete)
        }
    }
}

#[derive(Debug, Snafu)]
pub enum TriangulatorError {
    Fail,
    Incomplete,
}

pub(crate) struct Contour {
    pub(crate) points: Vec<Vec2>,
    pub(crate) identifier: usize,
    pub(crate) is_hole: bool,
    pub(crate) nested_to: Option<usize>,
}

fn ttf_outline_contour_info_majority(
    outline: &Triangulator,
    identifier: usize,
    contour: usize,
    nested_to: &mut Option<usize>,
) -> bool {
    let mut cont = [false; 3];
    let mut nto: [Option<usize>; 3] = [None; 3];
    let step = outline.contours[contour].points.len() / 3;
    cont[0] = ttf_outline_contour_info(outline, identifier, contour, step * 0, &mut nto[0]);
    cont[1] = ttf_outline_contour_info(outline, identifier, contour, step * 1, &mut nto[1]);
    cont[2] = ttf_outline_contour_info(outline, identifier, contour, step * 2, &mut nto[2]);
    let mut index = None;
    if nto[0].is_none() {
        index = Some(0)
    };
    if nto[1].is_none() {
        index = Some(1)
    };
    if nto[2].is_none() {
        index = Some(2)
    };
    if index.is_none() {
        index = if nto[0] == nto[1] || nto[0] == nto[2] {
            Some(0)
        } else {
            Some(1)
        };
    }
    *nested_to = nto[index.unwrap()];
    return cont[index.unwrap()];
}

fn ttf_outline_contour_info(
    outline: &Triangulator,
    identifier: usize,
    contour: usize,
    test_point: usize,
    nested_to: &mut Option<usize>,
) -> bool {
    let mut count = 0;
    let mut nto = None;
    let mut closest = 0.;
    for i in 0..outline.contours.len() {
        if i == contour {
            continue;
        }
        if outline.contours[i].identifier != identifier {
            continue;
        }
        let mut dist = 0.;
        let res = ttf_outline_evenodd_base(
            outline,
            &outline.contours[contour].points[test_point],
            i,
            &mut dist,
        );
        count += res;
        if (res & 1) == 0 {
            continue;
        };
        if nto.is_none() || dist < closest {
            closest = dist;
            nto = Some(i);
        }
    }
    *nested_to = nto;
    return (count & 1) == 0;
}

fn ttf_outline_evenodd_base(
    outline: &Triangulator,
    point: &Vec2,
    contour: usize,
    dist: &mut f32,
) -> i32 {
    let mut counter = 0;
    let mut closest_dx = 0.;

    let points = &outline.contours[contour].points;
    let len = outline.contours[contour].points.len();

    let mut prev = &points[len - 1];
    for pt in points.iter() {
        let (u, b): (&Vec2, &Vec2);

        if pt.y > prev.y {
            u = pt;
            b = prev;
        } else {
            u = prev;
            b = pt;
        }

        if point.y <= u.y && point.y > b.y {
            if point.x >= u.x || point.x >= b.x {
                //            u
                //      edge /
                //          / <-- dx --> .
                //         /            point
                //        b
                // calculation of distance from edge to point:
                // y(x) = b.y + (u.y - b.y) / (u.x - b.x) * (x - b.x)    - the edge line equation
                // x(y) = (y - b.y) / (u.y - b.y) * (u.x - b.x) + b.x    - inverse equation of line
                // dx = point.x - x(point.y)                             - final equation for distance
                let dy = u.y - b.y;
                if dy.abs() > EPSILON
                // horizontal edges are not handling
                {
                    let dx = point.x - (point.y - b.y) / dy * (u.x - b.x) - b.x;
                    if dx >= 0. {
                        if counter == 0 || dx < closest_dx {
                            closest_dx = dx;
                        }
                        counter += 1;
                    }
                }
            }
        }
        prev = pt;
    }

    *dist = closest_dx;

    counter
}
