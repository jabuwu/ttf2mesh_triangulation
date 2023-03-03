use macroquad::prelude::*;
use ttf2mesh_triangulation::Triangulator;

fn window_conf() -> Conf {
    Conf {
        window_title: "ttf2mesh triangulation".to_owned(),
        window_width: 1280,
        window_height: 720,
        high_dpi: true,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut world = World::new();
    let mut polygon_builder = PolygonBuilder::new();

    loop {
        clear_background(BLACK);

        world.update();
        set_camera(&world.camera());
        polygon_builder.update(&world);

        let mut triangulator = Triangulator::new();
        for vertex_list in polygon_builder.vertex_lists.iter() {
            let _ = triangulator.add_contour(0, vertex_list.iter().map(|v| [v.x, v.y]).collect());
        }
        let triangles = triangulator.triangulate().unwrap_or(vec![]);

        for triangle in triangles.iter() {
            draw_triangle(
                Vec2::from(triangle[0]),
                Vec2::from(triangle[1]),
                Vec2::from(triangle[2]),
                Color::new(1., 1., 1., 0.1),
            );
            for i in 0..3 {
                let j = (i + 1) % 3;
                draw_line(
                    triangle[i][0],
                    triangle[i][1],
                    triangle[j][0],
                    triangle[j][1],
                    1.,
                    WHITE,
                );
            }
        }

        set_default_camera();

        for (line, line_str) in [
            "Left click a vertex to drag it",
            "Left click an edge to create vertices",
            "Right click to create a new triangle",
            "WASD to pan camera",
        ]
        .into_iter()
        .enumerate()
        {
            draw_text_ex(
                line_str,
                10.,
                20. + (line as f32) * 20.,
                TextParams {
                    font_size: 40,
                    font_scale: 0.5,
                    ..Default::default()
                },
            );
        }

        next_frame().await
    }
}

struct World {
    pan: Vec2,
}

impl World {
    fn new() -> Self {
        Self { pan: Vec2::ZERO }
    }

    fn camera(&self) -> Camera2D {
        Camera2D {
            zoom: vec2(2. / screen_width(), -2. / screen_height()),
            offset: self.pan,
            ..Default::default()
        }
    }

    fn mouse_position(&self) -> Vec2 {
        let mut mouse_position = Vec2::from(mouse_position());
        let screen = Vec2::new(screen_width() * 0.5, screen_height() * 0.5);
        mouse_position.x -= screen.x + self.pan.x * screen.x;
        mouse_position.y -= screen.y - self.pan.y * screen.y;
        mouse_position
    }

    fn update(&mut self) {
        if is_key_down(KeyCode::W) {
            self.pan.y -= get_frame_time() * 0.5;
        }
        if is_key_down(KeyCode::S) {
            self.pan.y += get_frame_time() * 0.5;
        }
        if is_key_down(KeyCode::A) {
            self.pan.x += get_frame_time() * 0.5;
        }
        if is_key_down(KeyCode::D) {
            self.pan.x -= get_frame_time() * 0.5;
        }
    }
}

struct PolygonBuilder {
    state: PolygonBuilderState,
    vertex_lists: Vec<Vec<Vec2>>,
}

impl PolygonBuilder {
    fn new() -> Self {
        Self {
            state: PolygonBuilderState::None,
            vertex_lists: vec![
                vec![
                    Vec2::new(-200., -200.),
                    Vec2::new(200., -200.),
                    Vec2::new(200., 200.),
                    Vec2::new(-200., 200.),
                ],
                vec![
                    Vec2::new(-50., -50.),
                    Vec2::new(50., -50.),
                    Vec2::new(50., 50.),
                    Vec2::new(-50., 50.),
                ],
            ],
        }
    }

    fn update(&mut self, world: &World) {
        match self.state {
            PolygonBuilderState::None => {
                let closest_vertex_to_mouse = self.find_closest_vertex(world.mouse_position(), 20.);
                let closest_edge_to_mouse = if closest_vertex_to_mouse.is_none() {
                    self.find_closest_edge(world.mouse_position(), 20.)
                } else {
                    None
                };
                for (vertex_list_index, vertex_list) in self.vertex_lists.iter().enumerate() {
                    for (vertex_index, vertex) in vertex_list.iter().enumerate() {
                        let (color, radius) = if closest_vertex_to_mouse
                            .map(|(closest_vertex_list_index, closest_vertex_index)| {
                                closest_vertex_list_index == vertex_list_index
                                    && closest_vertex_index == vertex_index
                            })
                            .unwrap_or(false)
                        {
                            (YELLOW, 7.)
                        } else {
                            (WHITE, 3.)
                        };
                        draw_circle(vertex[0], vertex[1], radius, color);
                    }
                }
                if let Some((_, _, closest_edge_to_mouse_point)) = closest_edge_to_mouse {
                    draw_circle(
                        closest_edge_to_mouse_point.x,
                        closest_edge_to_mouse_point.y,
                        5.,
                        YELLOW,
                    );
                }
                if is_mouse_button_pressed(MouseButton::Left) {
                    if let Some((closest_vertex_to_mouse_list, closest_vertex_to_mouse)) =
                        closest_vertex_to_mouse
                    {
                        self.state = PolygonBuilderState::DraggingVertex(
                            closest_vertex_to_mouse_list,
                            closest_vertex_to_mouse,
                        );
                    } else if let Some((
                        closest_edge_to_mouse_list,
                        closest_edge_to_mouse,
                        closest_edge_to_mouse_point,
                    )) = closest_edge_to_mouse
                    {
                        self.vertex_lists[closest_edge_to_mouse_list]
                            .insert(closest_edge_to_mouse + 1, closest_edge_to_mouse_point);
                        self.state = PolygonBuilderState::DraggingVertex(
                            closest_edge_to_mouse_list,
                            closest_edge_to_mouse + 1,
                        );
                    }
                }
            }
            PolygonBuilderState::DraggingVertex(
                dragging_vertex_index_list,
                dragging_vertex_index,
            ) => {
                self.vertex_lists[dragging_vertex_index_list][dragging_vertex_index] =
                    world.mouse_position();
                for (vertex_index, vertex) in self.vertex_lists[dragging_vertex_index_list]
                    .iter()
                    .enumerate()
                {
                    if vertex_index == dragging_vertex_index {
                        draw_circle(vertex[0], vertex[1], 7., YELLOW);
                    }
                }
                if is_mouse_button_released(MouseButton::Left) {
                    self.state = PolygonBuilderState::None;
                }
            }
        }
        for vertex_list in self.vertex_lists.iter() {
            for i in 0..vertex_list.len() {
                let j = (i + 1) % vertex_list.len();
                draw_line(
                    vertex_list[i].x,
                    vertex_list[i].y,
                    vertex_list[j].x,
                    vertex_list[j].y,
                    3.,
                    WHITE,
                );
            }
        }
        if is_mouse_button_pressed(MouseButton::Right) {
            let mouse_position = world.mouse_position();
            self.vertex_lists.push(vec![
                Vec2::new(mouse_position.x + -10., mouse_position.y),
                Vec2::new(mouse_position.x + 10., mouse_position.y),
                Vec2::new(mouse_position.x, mouse_position.y - 20.),
            ]);
        }
    }

    fn find_closest_vertex(&self, point: Vec2, threshold: f32) -> Option<(usize, usize)> {
        self.find_closest_edge(point, threshold);
        let mut closest_vertex: Option<(usize, usize, f32)> = None;
        for (vertex_list_index, vertex_list) in self.vertex_lists.iter().enumerate() {
            for (vertex_index, vertex) in vertex_list.iter().enumerate() {
                let distance = Vec2::new(vertex[0], vertex[1]).distance(point);
                if distance < threshold {
                    if let Some(current_closest_vertex) = closest_vertex {
                        if distance < current_closest_vertex.2 {
                            closest_vertex = Some((vertex_list_index, vertex_index, distance));
                        }
                    } else {
                        closest_vertex = Some((vertex_list_index, vertex_index, distance));
                    }
                }
            }
        }
        closest_vertex.map(|(vertex_list_index, vertex_index, _)| (vertex_list_index, vertex_index))
    }

    fn find_closest_edge(&self, point: Vec2, threshold: f32) -> Option<(usize, usize, Vec2)> {
        let mut closest_edge: Option<(usize, usize, Vec2, f32)> = None;
        for (vertex_list_index, vertex_list) in self.vertex_lists.iter().enumerate() {
            for (vertex_index, vertex) in vertex_list.iter().enumerate() {
                let next_vertex_index = (vertex_index + 1) % vertex_list.len();
                let next_vertex = vertex_list[next_vertex_index];
                let line_direction = Vec2::new(next_vertex.x - vertex.x, next_vertex.y - vertex.y);
                let line_position = (Vec2::new(point.x - vertex.x, point.y - vertex.y)
                    .dot(line_direction)
                    / line_direction.dot(line_direction))
                .clamp(0., 1.);
                let closest_point = Vec2::new(
                    vertex.x + line_position * line_direction.x,
                    vertex.y + line_position * line_direction.y,
                );
                let distance = closest_point.distance(point);
                if distance < threshold {
                    if let Some(current_closest_vertex) = closest_edge {
                        if distance < current_closest_vertex.3 {
                            closest_edge =
                                Some((vertex_list_index, vertex_index, closest_point, distance));
                        }
                    } else {
                        closest_edge =
                            Some((vertex_list_index, vertex_index, closest_point, distance));
                    }
                }
            }
        }
        closest_edge.map(|(vertex_list_index, vertex_index, closest_point, _)| {
            (vertex_list_index, vertex_index, closest_point)
        })
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum PolygonBuilderState {
    None,
    DraggingVertex(usize, usize),
}
