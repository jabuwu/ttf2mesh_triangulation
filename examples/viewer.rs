use std::mem::{take, transmute};

use macroquad::prelude::*;
use ttf2mesh_triangulation::Triangulator;

const RESOLUTION: usize = 4;
const INVERSE_RESOLUTION: f32 = 1. / (RESOLUTION as f32);

fn window_conf() -> Conf {
    Conf {
        window_title: "ttf2mesh".to_owned(),
        window_width: 640,
        window_height: 200,
        high_dpi: true,
        ..Default::default()
    }
}

struct Character {
    triangles: Vec<[Vec2; 3]>,
    advance: Vec2,
}

#[macroquad::main(window_conf)]
async fn main() {
    let face = ttf_parser::Face::parse(include_bytes!("FiraSans-Bold.ttf"), 0).unwrap();
    let text = "ttf2mesh";
    let mut characters = vec![];
    let mut size = Vec2::ZERO;
    let scale = 0.15;
    for char in text.chars() {
        if let Some(glyph_id) = face.glyph_index(char) {
            let mut builder = TriangulatorBuilder::default();
            if let Some(bbox) = face.outline_glyph(glyph_id, &mut builder) {
                let mut triangles: Vec<[Vec2; 3]> =
                    unsafe { transmute(builder.triangulator.triangulate().unwrap()) };
                for triangle in triangles.iter_mut() {
                    for i in 0..3 {
                        triangle[i] *= scale;
                        triangle[i].y *= -1.;
                    }
                }
                characters.push(Character {
                    triangles,
                    advance: Vec2::new(
                        face.glyph_hor_advance(glyph_id).unwrap_or(0) as f32 * scale,
                        0.,
                    ),
                });
                size.x += face.glyph_hor_advance(glyph_id).unwrap_or(0) as f32 * scale;
                size.y = size.y.min((bbox.y_max as f32 - bbox.y_min as f32) * -scale) * 0.9;
            }
        }
    }

    loop {
        clear_background(BLACK);

        let mut position = Vec2::new(screen_width() * 0.5, screen_height() * 0.5) - size * 0.5;
        for character in characters.iter() {
            for triangle in character.triangles.iter() {
                for i in 0..3 {
                    let j = (i + 1) % 3;
                    draw_line(
                        position.x + triangle[i].x,
                        position.y + triangle[i].y,
                        position.x + triangle[j].x,
                        position.y + triangle[j].y,
                        1.,
                        WHITE,
                    );
                }
            }
            position += character.advance;
        }

        next_frame().await
    }
}

#[derive(Default)]
struct TriangulatorBuilder {
    contour: Vec<Vec2>,
    triangulator: Triangulator,
    has_contours: bool,
}

impl ttf_parser::OutlineBuilder for TriangulatorBuilder {
    fn move_to(&mut self, x: f32, y: f32) {
        if self.contour.len() > 0 {
            let _ = self
                .triangulator
                .add_contour(0, unsafe { transmute(take(&mut self.contour)) });
            self.contour = Vec::new();
            self.has_contours = true;
        }
        self.contour.push(Vec2::new(x, y));
    }

    fn line_to(&mut self, x: f32, y: f32) {
        self.contour.push(Vec2::new(x, y));
    }

    fn quad_to(&mut self, x1: f32, y1: f32, x: f32, y: f32) {
        if let Some(last) = self.contour.last() {
            let mut p0 = *last;
            let mut p1 = Vec2::new(x1, y1);
            let p2 = Vec2::new(x, y);
            let d01 = (p1 - p0) * INVERSE_RESOLUTION;
            let d12 = (p2 - p1) * INVERSE_RESOLUTION;
            for i in 0..(RESOLUTION - 1) {
                p0 += d01;
                p1 += d12;
                let cp = p0 + (p1 - p0) * (i as f32) * INVERSE_RESOLUTION;
                self.contour.push(cp);
            }
            self.contour.push(Vec2::new(x, y));
        } else {
            self.contour.push(Vec2::new(x, y));
        }
    }

    fn curve_to(&mut self, _x1: f32, _y1: f32, _x2: f32, _y2: f32, x: f32, y: f32) {
        // TODO
        self.contour.push(Vec2::new(x, y));
    }

    fn close(&mut self) {
        if self.contour.len() > 0 {
            let _ = self
                .triangulator
                .add_contour(0, unsafe { transmute(take(&mut self.contour)) });
            self.contour = Vec::new();
            self.has_contours = true;
        }
    }
}
