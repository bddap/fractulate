use anyhow::Result;
use nalgebra::Point3;
use std::io::{Cursor, Read};

use stl_io::{read_stl, write_stl, Normal, Triangle, Vertex};

fn main() -> Result<()> {
    let mesh = load()?;
    let mesh = transform(mesh);
    save(mesh)?;
    Ok(())
}

fn load() -> Result<Vec<[nalgebra::Point3<f32>; 3]>> {
    let mut buf = Vec::new();
    std::io::stdin().read_to_end(&mut buf).unwrap();
    let stl = read_stl(&mut Cursor::new(buf)).unwrap();

    let mesh = stl
        .faces
        .iter()
        .map(|face| {
            face.vertices.map(|v| {
                let ret: [f32; 3] = stl.vertices[v].into();
                let ret: Point3<f32> = ret.into();
                ret
            })
        })
        .collect();
    Ok(mesh)
}

fn save(mesh: Vec<[nalgebra::Point3<f32>; 3]>) -> Result<()> {
    let stl_io_mesh = mesh.into_iter().map(|triangle| {
        let vertices = triangle.map(|v| Vertex::new(v.into()));
        Triangle {
            normal: Normal::new(get_normal(&triangle).into()),
            vertices,
        }
    });
    write_stl(&mut std::io::stdout(), stl_io_mesh)?;
    Ok(())
}

fn transform(triangles: Vec<[nalgebra::Point3<f32>; 3]>) -> Vec<[nalgebra::Point3<f32>; 3]> {
    let mut mesh = triangles;
    for triangle in &mut mesh {
        for vertex in triangle[1..].iter_mut() {
            *vertex = nalgebra::Rotation3::from_euler_angles(0.0, 0.0, 0.001) * *vertex;
        }
    }
    mesh
}

fn get_normal(face: &[nalgebra::Point3<f32>; 3]) -> nalgebra::Vector3<f32> {
    let a = face[1] - face[0];
    let b = face[2] - face[0];
    a.cross(&b).normalize()
}
