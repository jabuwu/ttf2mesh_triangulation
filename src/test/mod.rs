use std::{
    io::Cursor,
    time::{Duration, Instant},
};

use byteorder::{LittleEndian, ReadBytesExt};

use crate::{Triangulator, Vec2};

#[test]
fn test_data() {
    let mut cursor = Cursor::new(include_bytes!("./test_data.bin"));
    let mut time = 0;
    let mut count = 0;
    loop {
        let next = cursor.read_i32::<LittleEndian>().unwrap();
        if next == 0 {
            break;
        }
        assert_eq!(next, 1);

        let label_length = cursor.read_i32::<LittleEndian>().unwrap();
        let mut label = String::new();
        for _ in 0..label_length {
            label.push(char::from_u32(cursor.read_u8().unwrap() as u32).unwrap());
        }
        dbg!(label);

        loop {
            let next = cursor.read_i32::<LittleEndian>().unwrap();
            if next == 0 {
                break;
            }
            assert_eq!(next, 1);

            let mut outline = Triangulator::new();

            let codepoint = cursor.read_i32::<LittleEndian>().unwrap();
            dbg!(codepoint);

            let contour_count = cursor.read_i32::<LittleEndian>().unwrap();
            for _ in 0..contour_count {
                let contour_length = cursor.read_i32::<LittleEndian>().unwrap();
                let contour_identifier = cursor.read_i32::<LittleEndian>().unwrap();

                let mut points = vec![];
                for _ in 0..contour_length {
                    let x = cursor.read_f32::<LittleEndian>().unwrap();
                    let y = cursor.read_f32::<LittleEndian>().unwrap();
                    points.push(Vec2::new(x, y));
                }

                outline
                    .add_contour(contour_identifier as usize, points)
                    .unwrap();
            }

            let now = Instant::now();
            let triangles = outline.triangulate().unwrap();
            time += now.elapsed().as_nanos();

            let triangle_count = cursor.read_i32::<LittleEndian>().unwrap();
            assert_eq!(triangles.len(), triangle_count as usize);
            for t in 0..(triangle_count as usize) {
                let v1x = cursor.read_f32::<LittleEndian>().unwrap();
                let v1y = cursor.read_f32::<LittleEndian>().unwrap();
                let v2x = cursor.read_f32::<LittleEndian>().unwrap();
                let v2y = cursor.read_f32::<LittleEndian>().unwrap();
                let v3x = cursor.read_f32::<LittleEndian>().unwrap();
                let v3y = cursor.read_f32::<LittleEndian>().unwrap();

                assert_eq!(triangles[t][0].x, v1x);
                assert_eq!(triangles[t][0].y, v1y);
                assert_eq!(triangles[t][1].x, v2x);
                assert_eq!(triangles[t][1].y, v2y);
                assert_eq!(triangles[t][2].x, v3x);
                assert_eq!(triangles[t][2].y, v3y);
            }

            count += 1;
        }
    }

    dbg!(Duration::from_nanos(time as u64));
    dbg!(count);
}
