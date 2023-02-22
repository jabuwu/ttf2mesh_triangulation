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

## Running the test

The only test included in this repo is one which checks byte-for-byte equality with ttf2mesh. It checks 11165 glyphs from two separate fonts. The test file is 100MB so it's not included in the git repo, and must be generated. More test fonts can be added easily in `test_gen/ttf2mesh_test_data_gen.c`.

```
cd test_gen
git clone https://github.com/fetisov/ttf2mesh.git
cd ttf2mesh
git checkout 6caa474b3f0567db6d3a35997b96c27fd87ba439
cd ..
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RELEASE
make
./ttf2mesh_test_data_gen
cd ../..
cargo test --release
```
