# ttf2mesh_triangulation

Triangulation code ripped from [ttf2mesh](https://github.com/fetisov/ttf2mesh) and ported to Rust.

Provides fixes for the original ttf2mesh algorithm, or produces the same exact results if using the `original_algorithm` feature.

## Example

```rust
use ttf2mesh_triangulation::Triangulator;

fn main() {
    let mut triangulator = Triangulator::new();

    triangulator
        .add_contour(
            0,
            vec![
                [-100., -100.],
                [100., -100.],
                [100., 100.],
                [-100., 100.]
            ],
        )
        .unwrap();

    triangulator
        .add_contour(
            0,
            vec![
                [-50., -50.],
                [50., -50.],
                [50., 50.],
                [-50., 50.]
            ]
        )
        .unwrap();

    let triangles: Vec<[[f32; 2]; 3]> = triangulator.triangulate().unwrap();
    dbg!(triangles);
}
```

## Running the ttf2mesh equality test

A test included in this repo checks byte-for-byte equality with ttf2mesh. It checks 11165 glyphs from two separate fonts. The test file is 100MB so it's not included in the git repo, and must be generated. More test fonts can be added easily in `test_gen/ttf2mesh_test_data_gen.c`.

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
cargo test --release --features original_algorithm
```
