//! Scripted orbit around the candi.
//!
//! The drone flies concentric rings at several heights, always facing inward.
//! Motion is kinematic: each waypoint is written straight to the mocap body.

/// One pose on the orbit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Waypoint {
    /// World position of the drone body.
    pub pos: [f64; 3],
    /// Body orientation as a MuJoCo quaternion, [w, x, y, z].
    pub quat: [f64; 4],
    /// Ring index this waypoint belongs to, for logging.
    pub ring: usize,
}

/// A full multi-ring orbit.
#[derive(Debug, Clone)]
pub struct OrbitPlan {
    pub waypoints: Vec<Waypoint>,
    /// Point the drone aims at.
    pub centre: [f64; 3],
    /// Ring radius actually used.
    pub radius: f64,
    pub heights: Vec<f64>,
}

impl OrbitPlan {
    /// Build an orbit around `centre`.
    ///
    /// `radius_factor` scales the bounding sphere implied by `bbox_dims`, so
    /// 1.5 keeps the drone one half-radius clear of the structure.
    /// `heights` are absolute world z values.
    pub fn generate(
        centre: [f64; 3],
        bbox_dims: [f64; 3],
        heights: &[f64],
        radius_factor: f64,
        points_per_ring: usize,
    ) -> Self {
        assert!(points_per_ring >= 3, "a ring needs at least 3 points");
        assert!(!heights.is_empty(), "need at least one ring height");

        let bounding_sphere =
            0.5 * (bbox_dims[0].powi(2) + bbox_dims[1].powi(2) + bbox_dims[2].powi(2)).sqrt();
        let radius = radius_factor * bounding_sphere;

        let mut waypoints = Vec::with_capacity(heights.len() * points_per_ring);

        for (ring, &height) in heights.iter().enumerate() {
            for i in 0..points_per_ring {
                // Alternate direction per ring so the drone does not have to
                // fly all the way back around between rings.
                let step = i as f64 / points_per_ring as f64;
                let frac = if ring % 2 == 0 { step } else { 1.0 - step };
                let theta = frac * std::f64::consts::TAU;

                let pos = [
                    centre[0] + radius * theta.cos(),
                    centre[1] + radius * theta.sin(),
                    height,
                ];
                waypoints.push(Waypoint {
                    pos,
                    quat: look_at_quat(pos, centre),
                    ring,
                });
            }
        }

        Self {
            waypoints,
            centre,
            radius,
            heights: heights.to_vec(),
        }
    }

    /// Ring heights spread over a structure spanning `z_lo..z_hi`.
    ///
    /// The rings sit at 25/50/75/105% of the height: the top one deliberately
    /// overshoots so the drone looks down on the summit, which is otherwise
    /// only ever seen edge-on.
    pub fn heights_for(z_lo: f64, z_hi: f64, count: usize) -> Vec<f64> {
        let span = z_hi - z_lo;
        (0..count)
            .map(|i| {
                let frac = (i + 1) as f64 / count as f64;
                z_lo + span * frac * 1.05
            })
            .collect()
    }

    pub fn len(&self) -> usize {
        self.waypoints.len()
    }

    pub fn is_empty(&self) -> bool {
        self.waypoints.is_empty()
    }
}

/// Quaternion that aims the body's local +X axis from `from` towards `to`,
/// keeping the body's +Z as close to world up as possible.
///
/// The +X convention comes from the MJCF: `drone_cam` is declared with
/// `xyaxes="0 -1 0  0 0 1"`, which points the camera's view direction along
/// the body's +X. Aiming +X at the candi therefore aims the camera at it.
pub fn look_at_quat(from: [f64; 3], to: [f64; 3]) -> [f64; 4] {
    let mut x = normalize([to[0] - from[0], to[1] - from[1], to[2] - from[2]]);
    if x == [0.0, 0.0, 0.0] {
        x = [1.0, 0.0, 0.0];
    }

    const WORLD_UP: [f64; 3] = [0.0, 0.0, 1.0];

    // y = up x forward, giving a horizontal axis perpendicular to the view.
    let mut y = cross(WORLD_UP, x);
    if norm(y) < 1e-9 {
        // Looking straight up or down: any perpendicular will do.
        y = cross([0.0, 1.0, 0.0], x);
    }
    let y = normalize(y);
    let z = cross(x, y);

    // Columns of the rotation matrix are the body axes in world coordinates.
    mat_to_quat([x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]])
}

/// Row-major 3x3 rotation matrix to a MuJoCo [w, x, y, z] quaternion.
///
/// Uses the largest-diagonal branch so the divisor never approaches zero.
fn mat_to_quat(m: [f64; 9]) -> [f64; 4] {
    let (m00, m01, m02) = (m[0], m[1], m[2]);
    let (m10, m11, m12) = (m[3], m[4], m[5]);
    let (m20, m21, m22) = (m[6], m[7], m[8]);

    let trace = m00 + m11 + m22;

    let q = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        [0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s]
    } else if m00 > m11 && m00 > m22 {
        let s = (1.0 + m00 - m11 - m22).sqrt() * 2.0;
        [(m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s]
    } else if m11 > m22 {
        let s = (1.0 + m11 - m00 - m22).sqrt() * 2.0;
        [(m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s]
    } else {
        let s = (1.0 + m22 - m00 - m11).sqrt() * 2.0;
        [(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s]
    };

    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn norm(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

fn normalize(v: [f64; 3]) -> [f64; 3] {
    let n = norm(v);
    if n < 1e-12 {
        [0.0, 0.0, 0.0]
    } else {
        [v[0] / n, v[1] / n, v[2] / n]
    }
}

/// Rotate `v` by the MuJoCo quaternion `q` = [w, x, y, z].
pub fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let t = [
        2.0 * (y * v[2] - z * v[1]),
        2.0 * (z * v[0] - x * v[2]),
        2.0 * (x * v[1] - y * v[0]),
    ];
    [
        v[0] + w * t[0] + (y * t[2] - z * t[1]),
        v[1] + w * t[1] + (z * t[0] - x * t[2]),
        v[2] + w * t[2] + (x * t[1] - y * t[0]),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    const CENTRE: [f64; 3] = [0.0, 0.0, 3.0];

    #[test]
    fn ring_points_sit_on_a_circle() {
        let plan = OrbitPlan::generate(CENTRE, [14.95, 14.95, 6.0], &[3.0], 1.5, 72);
        assert_eq!(plan.len(), 72);

        for wp in &plan.waypoints {
            let dx = wp.pos[0] - CENTRE[0];
            let dy = wp.pos[1] - CENTRE[1];
            let r = (dx * dx + dy * dy).sqrt();
            assert!(
                (r - plan.radius).abs() < 1e-9,
                "waypoint at radius {r}, expected {}",
                plan.radius
            );
            assert!((wp.pos[2] - 3.0).abs() < 1e-9);
        }
    }

    #[test]
    fn radius_follows_the_bounding_sphere() {
        let dims = [14.95, 14.95, 6.0];
        let plan = OrbitPlan::generate(CENTRE, dims, &[3.0], 1.5, 8);
        let expected = 1.5 * 0.5 * (dims[0].powi(2) + dims[1].powi(2) + dims[2].powi(2)).sqrt();
        assert!((plan.radius - expected).abs() < 1e-9);
        // Sanity-check against the measured scene: 16.48 m.
        assert!(
            (plan.radius - 16.48).abs() < 0.02,
            "radius was {}",
            plan.radius
        );
    }

    #[test]
    fn every_waypoint_faces_the_centre() {
        let plan = OrbitPlan::generate(CENTRE, [14.95, 14.95, 6.0], &[1.5, 6.3], 1.5, 36);

        for wp in &plan.waypoints {
            // The body's +X axis must point at the centre.
            let forward = rotate(wp.quat, [1.0, 0.0, 0.0]);
            let to_centre = normalize([
                CENTRE[0] - wp.pos[0],
                CENTRE[1] - wp.pos[1],
                CENTRE[2] - wp.pos[2],
            ]);
            for axis in 0..3 {
                assert!(
                    (forward[axis] - to_centre[axis]).abs() < 1e-9,
                    "ring {} facing {forward:?}, expected {to_centre:?}",
                    wp.ring
                );
            }
        }
    }

    #[test]
    fn quaternions_are_unit_and_upright() {
        let plan = OrbitPlan::generate(CENTRE, [14.95, 14.95, 6.0], &[6.3], 1.5, 16);
        for wp in &plan.waypoints {
            let n = wp.quat.iter().map(|c| c * c).sum::<f64>().sqrt();
            assert!((n - 1.0).abs() < 1e-9, "quat not unit: {n}");

            // Body +Z should keep a positive world-up component: the drone
            // banks to look down but never rolls over.
            let up = rotate(wp.quat, [0.0, 0.0, 1.0]);
            assert!(up[2] > 0.0, "drone is upside down: {up:?}");
        }
    }

    #[test]
    fn rings_alternate_direction() {
        let plan = OrbitPlan::generate(CENTRE, [10.0, 10.0, 5.0], &[2.0, 4.0], 1.5, 4);
        // Ring 0 runs counter-clockwise from theta=0, ring 1 clockwise.
        assert!(plan.waypoints[1].pos[1] > plan.waypoints[0].pos[1]);
        assert!(plan.waypoints[5].pos[1] < plan.waypoints[4].pos[1]);
    }

    #[test]
    fn heights_span_the_structure() {
        let h = OrbitPlan::heights_for(0.0, 6.0, 4);
        assert_eq!(h.len(), 4);
        assert!(h[0] > 0.0 && h[0] < 2.0);
        // The top ring clears the summit so the drone can look down on it.
        assert!(*h.last().unwrap() > 6.0, "top ring at {:?}", h.last());
    }
}
