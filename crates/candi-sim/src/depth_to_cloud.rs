//! Turn a MuJoCo depth buffer into a world-frame point cloud.

/// World pose of the rendering camera.
#[derive(Debug, Clone, Copy)]
pub struct CameraPose {
    /// Camera origin in world coordinates (`data.cam_xpos()`).
    pub pos: [f64; 3],
    /// Row-major 3x3 camera-to-world rotation (`data.cam_xmat()`).
    ///
    /// Taken straight from MuJoCo rather than rebuilt from the drone's pose
    /// and the camera's MJCF offset — that keeps one authority for the frame
    /// convention instead of two that can drift apart.
    pub mat: [f64; 9],
}

impl CameraPose {
    fn to_world(self, p: [f64; 3]) -> [f32; 3] {
        [
            (self.pos[0] + self.mat[0] * p[0] + self.mat[1] * p[1] + self.mat[2] * p[2]) as f32,
            (self.pos[1] + self.mat[3] * p[0] + self.mat[4] * p[1] + self.mat[5] * p[2]) as f32,
            (self.pos[2] + self.mat[6] * p[0] + self.mat[7] * p[1] + self.mat[8] * p[2]) as f32,
        ]
    }
}

/// Pinhole intrinsics, derived from the MJCF camera.
#[derive(Debug, Clone, Copy)]
pub struct Intrinsics {
    pub width: usize,
    pub height: usize,
    /// Focal length in pixels; MuJoCo's `fovy` is the *vertical* field of view.
    pub focal: f64,
}

impl Intrinsics {
    /// `fovy_deg` is the MJCF camera's `fovy` attribute.
    pub fn from_fovy(width: usize, height: usize, fovy_deg: f64) -> Self {
        let focal = (height as f64 / 2.0) / (fovy_deg.to_radians() / 2.0).tan();
        Self {
            width,
            height,
            focal,
        }
    }
}

/// Unproject a MuJoCo depth buffer into world-frame points.
///
/// Three properties of the buffer, all confirmed by the smoke-test example,
/// are relied on here:
///
/// * Values are already **metres** — `MjRenderer::render` linearizes them, so
///   there is no znear/zfar maths to redo.
/// * Row 0 is the **top** of the image; the renderer flips OpenGL's bottom-up
///   readback for us.
/// * A value is **z-depth along the view axis**, not a range along the ray.
///   Hence `x = (u - cx) * z / f` rather than scaling a unit direction — a
///   flat wall reads a constant depth across the whole frame, and treating it
///   as range would bow the cloud outwards into a bowl.
///
/// Background pixels come back at roughly the far plane, *not* at zero, so the
/// far cut is what removes the sky. The near cut only guards against
/// degenerate values.
///
/// `subsample` of 4 keeps every 4th pixel on both axes, i.e. 1/16 of them.
pub fn depth_to_cloud(
    depth: &[f32],
    intrinsics: Intrinsics,
    camera: CameraPose,
    max_range: f32,
    subsample: usize,
) -> Vec<[f32; 3]> {
    let Intrinsics {
        width,
        height,
        focal,
    } = intrinsics;

    assert_eq!(
        depth.len(),
        width * height,
        "depth buffer is {} values, expected {}",
        depth.len(),
        width * height
    );
    let step = subsample.max(1);

    let cx = width as f64 / 2.0;
    let cy = height as f64 / 2.0;

    let mut cloud = Vec::with_capacity((width / step) * (height / step));

    for v in (0..height).step_by(step) {
        for u in (0..width).step_by(step) {
            let z = depth[v * width + u];
            if !z.is_finite() || z <= 1e-3 || z >= max_range {
                continue;
            }

            let z = z as f64;
            // Camera looks along its own -Z, with +X right and +Y up, while
            // image rows run downwards — hence the negated y and z.
            let p = [
                (u as f64 - cx) * z / focal,
                -(v as f64 - cy) * z / focal,
                -z,
            ];
            cloud.push(camera.to_world(p));
        }
    }

    cloud
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Camera at the origin looking down world -Z, +X right, +Y up: the
    /// identity orientation, so camera and world frames coincide.
    fn identity_camera() -> CameraPose {
        CameraPose {
            pos: [0.0, 0.0, 0.0],
            mat: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        }
    }

    #[test]
    fn centre_pixel_maps_along_the_view_axis() {
        let intr = Intrinsics::from_fovy(8, 8, 60.0);
        let depth = vec![5.0f32; 64];
        let cloud = depth_to_cloud(&depth, intr, identity_camera(), 10.0, 4);

        // With width 8 and step 4 the sampled columns are 0 and 4; column 4 is
        // the centre (cx = 4), and row 4 likewise.
        let centre = cloud
            .iter()
            .find(|p| p[0].abs() < 1e-6 && p[1].abs() < 1e-6)
            .expect("no point on the optical axis");
        assert!((centre[2] - (-5.0)).abs() < 1e-5, "got {centre:?}");
    }

    #[test]
    fn a_flat_wall_stays_flat() {
        // Constant z-depth must produce a plane, not a bowl. This is the
        // regression guard for treating depth as range.
        let intr = Intrinsics::from_fovy(64, 48, 60.0);
        let depth = vec![4.0f32; 64 * 48];
        let cloud = depth_to_cloud(&depth, intr, identity_camera(), 10.0, 4);

        assert!(!cloud.is_empty());
        for p in &cloud {
            assert!(
                (p[2] - (-4.0)).abs() < 1e-5,
                "point off the plane: {p:?} (bowing means depth was treated as range)"
            );
        }
    }

    #[test]
    fn horizontal_extent_matches_the_field_of_view() {
        let (w, h, fovy) = (64, 48, 60.0);
        let intr = Intrinsics::from_fovy(w, h, fovy);
        let z = 4.0f64;
        let depth = vec![z as f32; w * h];
        let cloud = depth_to_cloud(&depth, intr, identity_camera(), 10.0, 1);

        // Vertical half-extent is z * tan(fovy/2); horizontal scales by aspect.
        let half_y = z * (fovy.to_radians() / 2.0).tan();
        let half_x = half_y * w as f64 / h as f64;

        let max_x = cloud.iter().map(|p| p[0]).fold(f32::MIN, f32::max) as f64;
        let max_y = cloud.iter().map(|p| p[1]).fold(f32::MIN, f32::max) as f64;

        // Sampling stops one pixel short of the edge, hence the tolerance.
        assert!(
            (max_x - half_x).abs() < 0.2,
            "max_x {max_x}, expected ~{half_x}"
        );
        assert!(
            (max_y - half_y).abs() < 0.2,
            "max_y {max_y}, expected ~{half_y}"
        );
    }

    #[test]
    fn background_and_invalid_pixels_are_dropped() {
        let intr = Intrinsics::from_fovy(4, 4, 60.0);
        let mut depth = vec![5.0f32; 16];
        depth[0] = 60.0; // sky, sits near the far plane
        depth[1] = 0.0; // degenerate
        depth[2] = f32::NAN;
        depth[3] = 10.0; // exactly at max_range

        let cloud = depth_to_cloud(&depth, intr, identity_camera(), 10.0, 1);
        assert_eq!(cloud.len(), 16 - 4);
    }

    #[test]
    fn subsampling_keeps_every_nth_pixel() {
        let intr = Intrinsics::from_fovy(64, 48, 60.0);
        let depth = vec![3.0f32; 64 * 48];

        let full = depth_to_cloud(&depth, intr, identity_camera(), 10.0, 1);
        let quarter = depth_to_cloud(&depth, intr, identity_camera(), 10.0, 4);

        assert_eq!(full.len(), 64 * 48);
        assert_eq!(quarter.len(), (64 / 4) * (48 / 4));
    }

    #[test]
    fn camera_pose_moves_points_into_the_world() {
        let intr = Intrinsics::from_fovy(4, 4, 60.0);
        let depth = vec![2.0f32; 16];

        // The drone_cam arrangement: camera at (10, 0, 3) looking horizontally
        // back at the origin, world +Z up. Columns of `mat` are the camera
        // axes in world coordinates — +Z maps to world +X so the view
        // direction (camera -Z) runs along world -X, and +Y maps to world +Z.
        let camera = CameraPose {
            pos: [10.0, 0.0, 3.0],
            mat: [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        };
        let cloud = depth_to_cloud(&depth, intr, camera, 10.0, 1);

        for p in &cloud {
            assert!((p[0] - 8.0).abs() < 1e-4, "expected x=8, got {p:?}");
        }
    }
}
