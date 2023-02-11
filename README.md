# ttf2mesh_triangulation

Triangulation code ripped from [ttf2mesh](https://github.com/fetisov/ttf2mesh) and ported to Rust. Produces the same exact results as ttf2mesh.

## Example

```rust
use glam::Vec2;
use ttf2mesh_triangulation::Triangulator;

let mut triangulator = Triangulator::new();
triangulator.add_contour(
    0,
    vec![
        Vec2::new(-100., -100.),
        Vec2::new(100., -100.),
        Vec2::new(100., 100.),
        Vec2::new(-100., 100.),
    ],
);
triangulator.add_contour(
    0,
    vec![
        Vec2::new(-50., -50.),
        Vec2::new(50., -50.),
        Vec2::new(50., 50.),
        Vec2::new(-50., 50.),
    ],
);
let triangles: Vec<[Vec2; 3]> = triangulator.triangulate().unwrap();
```
