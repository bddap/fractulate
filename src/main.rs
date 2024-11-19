use anyhow::Result;
use nalgebra::{Matrix4, Vector3};
use rand::{Rng, SeedableRng};
use std::io::{Cursor, Read};

use stl_io::{read_stl, write_stl, Normal, Triangle, Vertex};

fn main() -> Result<()> {
    let mesh = load()?;
    let mesh = main_transform(mesh);
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

fn main_transform(triangles: Vec<[Vector3<f32>; 3]>) -> Vec<[Vector3<f32>; 3]> {
    let mut rng = rand_xoshiro::Xoshiro256StarStar::seed_from_u64(0);
    let mut ret = triangles.clone();
    ret.extend(growths(&mut rng, &triangles, 2));
    ret
}

fn growths<R: Rng>(
    rng: &mut R,
    base_model: &[[Vector3<f32>; 3]],
    depth: usize,
) -> Vec<[Vector3<f32>; 3]> {
    let Some(next_depth) = depth.checked_sub(1) else {
        return Vec::new();
    };

    let num_children = 5;
    let child_scale = 0.5;

    let mut ret = Vec::new();

    for _ in 0..num_children {
        let triangle = select(rng, base_model);
        let transformation = place_on_triangle(triangle) * Matrix4::new_scaling(child_scale);
        ret.extend(transform(base_model.to_vec(), transformation));
        ret.extend(transform(
            growths(rng, base_model, next_depth),
            transformation,
        ));
    }

    ret
}

/// Choose a random triangle, weighted by its area.
// this could be sped up with some precomputation and a binary search but yolo
fn select<R: Rng>(rng: &mut R, triangles: &[[Vector3<f32>; 3]]) -> [Vector3<f32>; 3] {
    assert!(!triangles.is_empty());

    let areas = triangles
        .iter()
        .map(|triangle| {
            let [t0, t1, t2] = triangle;
            let a = t1 - t0;
            let b = t2 - t0;
            a.cross(&b).norm()
        })
        .collect::<Vec<_>>();
    let total_area = areas.iter().sum::<f32>();
    let mut area = rng.gen_range(0.0..total_area);
    for (i, &a) in areas.iter().enumerate() {
        if area < a {
            return triangles[i];
        }
        area -= a;
    }

    // probably floating point error, return the last triangle
    triangles[triangles.len() - 1]
}

fn transform(
    triangles: Vec<[Vector3<f32>; 3]>,
    transformation: Matrix4<f32>,
) -> Vec<[Vector3<f32>; 3]> {
    let mut triangles = triangles;
    for triangle in triangles.iter_mut() {
        for v in triangle.iter_mut() {
            *v = transformation.transform_point(&(*v).into()).coords;
        }
    }
    triangles
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
