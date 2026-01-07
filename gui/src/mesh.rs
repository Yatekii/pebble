//! 3D mesh loading and data structures for GLB/glTF files.

use std::collections::HashSet;

/// A 3D vertex position.
#[derive(Clone, Copy, Debug)]
pub struct Vertex {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vertex {
    /// Transform vertex by a 4x4 matrix (column-major).
    fn transform(self, matrix: &[[f32; 4]; 4]) -> Self {
        let x =
            matrix[0][0] * self.x + matrix[1][0] * self.y + matrix[2][0] * self.z + matrix[3][0];
        let y =
            matrix[0][1] * self.x + matrix[1][1] * self.y + matrix[2][1] * self.z + matrix[3][1];
        let z =
            matrix[0][2] * self.x + matrix[1][2] * self.y + matrix[2][2] * self.z + matrix[3][2];
        Self { x, y, z }
    }

    /// Convert from Y-up (glTF/KiCad) to Z-up coordinate system.
    /// glTF uses Y-up, but we want Z-up (Z perpendicular to PCB).
    /// First convert Y-up to Z-up, then rotate 180° around X to flip the PCB.
    fn convert_y_up_to_z_up(self) -> Self {
        // Step 1: Y-up to Z-up conversion (swap Y and Z)
        let x = self.x;
        let y = -self.z;
        let z = self.y;

        // Step 2: Rotate 180° around X-axis to flip PCB right-side up
        // Rotation matrix for 180° around X: [[1,0,0], [0,-1,0], [0,0,-1]]
        Self { x, y: -y, z: -z }
    }
}

/// An edge between two vertex indices.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Edge(pub usize, pub usize);

impl Edge {
    /// Create a normalized edge (smaller index first) for deduplication.
    pub fn new(a: usize, b: usize) -> Self {
        if a < b { Edge(a, b) } else { Edge(b, a) }
    }
}

/// A loaded 3D mesh optimized for wireframe rendering.
#[derive(Clone, Debug)]
pub struct Mesh3D {
    /// All vertex positions.
    pub vertices: Vec<Vertex>,
    /// Unique edges for wireframe rendering.
    pub edges: Vec<Edge>,
    /// Bounding box min.
    pub bounds_min: Vertex,
    /// Bounding box max.
    pub bounds_max: Vertex,
}

/// Multiply two 4x4 matrices (column-major).
fn mat4_multiply(a: &[[f32; 4]; 4], b: &[[f32; 4]; 4]) -> [[f32; 4]; 4] {
    let mut result = [[0.0f32; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            result[i][j] =
                a[0][j] * b[i][0] + a[1][j] * b[i][1] + a[2][j] * b[i][2] + a[3][j] * b[i][3];
        }
    }
    result
}

/// Identity matrix.
fn mat4_identity() -> [[f32; 4]; 4] {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

impl Mesh3D {
    /// Load a mesh from GLB binary data.
    pub fn from_glb(data: &[u8]) -> anyhow::Result<Self> {
        let (document, buffers, _images) = gltf::import_slice(data)?;

        let mut all_vertices = Vec::new();
        let mut edge_set = HashSet::new();

        // Process all scenes and nodes to get proper transforms
        for scene in document.scenes() {
            for node in scene.nodes() {
                Self::process_node(
                    &node,
                    &buffers,
                    &mat4_identity(),
                    &mut all_vertices,
                    &mut edge_set,
                );
            }
        }

        // Calculate bounding box
        let (bounds_min, bounds_max) = if all_vertices.is_empty() {
            (
                Vertex {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                Vertex {
                    x: 1.0,
                    y: 1.0,
                    z: 1.0,
                },
            )
        } else {
            let mut min = Vertex {
                x: f32::MAX,
                y: f32::MAX,
                z: f32::MAX,
            };
            let mut max = Vertex {
                x: f32::MIN,
                y: f32::MIN,
                z: f32::MIN,
            };
            for v in &all_vertices {
                min.x = min.x.min(v.x);
                min.y = min.y.min(v.y);
                min.z = min.z.min(v.z);
                max.x = max.x.max(v.x);
                max.y = max.y.max(v.y);
                max.z = max.z.max(v.z);
            }
            (min, max)
        };

        let edges: Vec<Edge> = edge_set.into_iter().collect();

        eprintln!(
            "Loaded mesh: {} vertices, {} edges",
            all_vertices.len(),
            edges.len()
        );
        eprintln!(
            "Bounds: ({:.2}, {:.2}, {:.2}) to ({:.2}, {:.2}, {:.2})",
            bounds_min.x, bounds_min.y, bounds_min.z, bounds_max.x, bounds_max.y, bounds_max.z
        );

        Ok(Mesh3D {
            vertices: all_vertices,
            edges,
            bounds_min,
            bounds_max,
        })
    }

    /// Process a node and its children recursively, applying transforms.
    fn process_node(
        node: &gltf::Node,
        buffers: &[gltf::buffer::Data],
        parent_transform: &[[f32; 4]; 4],
        all_vertices: &mut Vec<Vertex>,
        edge_set: &mut HashSet<Edge>,
    ) {
        // Get this node's local transform matrix
        let local_transform = node.transform().matrix();
        // Convert from [[f32; 4]; 4] to our format (it's the same, just need to copy)
        let local_matrix: [[f32; 4]; 4] = local_transform;

        // Combine with parent transform
        let world_transform = mat4_multiply(parent_transform, &local_matrix);

        // Process mesh if this node has one
        if let Some(mesh) = node.mesh() {
            for primitive in mesh.primitives() {
                let reader = primitive.reader(|buffer| Some(&buffers[buffer.index()]));

                // Get vertex positions and transform them
                let positions: Vec<Vertex> = reader
                    .read_positions()
                    .map(|iter| {
                        iter.map(|[x, y, z]| {
                            let v = Vertex { x, y, z };
                            v.transform(&world_transform).convert_y_up_to_z_up()
                        })
                        .collect::<Vec<_>>()
                    })
                    .unwrap_or_default();

                if positions.is_empty() {
                    continue;
                }

                let vertex_offset = all_vertices.len();

                // Get triangle indices and extract edges
                if let Some(indices) = reader.read_indices() {
                    let indices: Vec<usize> = indices.into_u32().map(|i| i as usize).collect();

                    // Process triangles (every 3 indices)
                    for chunk in indices.chunks(3) {
                        if chunk.len() == 3 {
                            let a = vertex_offset + chunk[0];
                            let b = vertex_offset + chunk[1];
                            let c = vertex_offset + chunk[2];

                            edge_set.insert(Edge::new(a, b));
                            edge_set.insert(Edge::new(b, c));
                            edge_set.insert(Edge::new(c, a));
                        }
                    }
                }

                all_vertices.extend(positions);
            }
        }

        // Process children recursively
        for child in node.children() {
            Self::process_node(&child, buffers, &world_transform, all_vertices, edge_set);
        }
    }

    /// Get the center of the mesh.
    pub fn center(&self) -> Vertex {
        Vertex {
            x: (self.bounds_min.x + self.bounds_max.x) / 2.0,
            y: (self.bounds_min.y + self.bounds_max.y) / 2.0,
            z: (self.bounds_min.z + self.bounds_max.z) / 2.0,
        }
    }

    /// Get the maximum dimension (for scaling).
    pub fn max_dimension(&self) -> f32 {
        let dx = self.bounds_max.x - self.bounds_min.x;
        let dy = self.bounds_max.y - self.bounds_min.y;
        let dz = self.bounds_max.z - self.bounds_min.z;
        dx.max(dy).max(dz)
    }
}
