use anyhow::Result;
use nalgebra::{Matrix4, Point3, Vector3};
use rand::{seq::SliceRandom, SeedableRng};
use std::io::{Cursor, Read};

use stl_io::{read_stl, write_stl, Normal, Triangle, Vertex};

fn main() -> Result<()> {
    let mesh = load()?;
    let mesh = transform(mesh);
    save(mesh)?;
    Ok(())
}

fn load() -> Result<Vec<[Vector3<f32>; 3]>> {
    let mut buf = Vec::new();
    std::io::stdin().read_to_end(&mut buf).unwrap();
    let stl = read_stl(&mut Cursor::new(buf)).unwrap();

    let mesh = stl
        .faces
        .iter()
        .map(|face| {
            face.vertices.map(|v| {
                let ret: [f32; 3] = stl.vertices[v].into();
                let ret: Vector3<f32> = ret.into();
                ret
            })
        })
        .collect();
    Ok(mesh)
}

fn save(mesh: Vec<[Vector3<f32>; 3]>) -> Result<()> {
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

fn transform(triangles: Vec<[Vector3<f32>; 3]>) -> Vec<[Vector3<f32>; 3]> {
    let mut mesh = triangles.clone();

    let mut rng = rand_xoshiro::Xoshiro256StarStar::seed_from_u64(0);

    for _ in 0..30 {
        let Some(triangle) = triangles.choose(&mut rng) else {
            return Vec::new();
        };
        let transformation = place_on_triangle(*triangle) * Matrix4::new_scaling(0.5);
        mesh.extend(triangles.iter().map(|triangle| {
            triangle.map(|v| {
                let v: Point3<f32> = v.into();
                let v = transformation.transform_point(&v);
                v.coords
            })
        }));
    }

    mesh
}

fn get_normal(face: &[Vector3<f32>; 3]) -> nalgebra::Vector3<f32> {
    let a = face[1] - face[0];
    let b = face[2] - face[0];
    a.cross(&b).normalize()
}

/// Create a transformation that would move a mesh so it sticks out from the triangle.
pub fn place_on_triangle(triangle: [Vector3<f32>; 3]) -> Matrix4<f32> {
    let [v0, v1, v2] = triangle;

    let normal = get_normal(&triangle);
    let x_axis = (v1 - v0).normalize();
    let y_axis = normal.cross(&x_axis);
    let rotation = Matrix4::new(
        x_axis.x, y_axis.x, normal.x, 0.0, x_axis.y, y_axis.y, normal.y, 0.0, x_axis.z, y_axis.z,
        normal.z, 0.0, 0.0, 0.0, 0.0, 1.0,
    );

    let center = (v0 + v1 + v2) / 3.0;
    let translation = Matrix4::new_translation(&center);

    translation * rotation
}
