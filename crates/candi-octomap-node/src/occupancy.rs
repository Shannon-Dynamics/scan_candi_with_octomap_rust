//! Probabilistic occupancy mapping, OctoMap-style, on a hash grid.
//!
//! This is **not** the mapping library this repository demonstrates — that is
//! `octomap-core`, and both maps run side by side on the same points. This one
//! is kept as an independently written implementation to check the octree
//! against: two maps built from different data structures agreeing on a voxel
//! count is worth more than either asserting it alone. See
//! `docs/decisions/0009-dual-map-comparison.md`.
//!
//! It keeps OctoMap's probabilistic semantics — log-odds updates, clamping,
//! and free-space carving along each ray — but stores voxels in a hash map
//! keyed by integer grid coordinates rather than in an octree. At this scene's
//! scale (a ~40 m box at 0.1 m resolution, of which only surfaces are ever
//! touched) the hash map is simpler to get right and cheap to query; the
//! octree's advantage is memory compaction over far larger volumes, which is
//! what free-space carving turns into a decisive difference.

use std::collections::HashMap;

/// Integer voxel coordinate.
pub type Key = (i32, i32, i32);

/// OctoMap's default sensor model.
pub const PROB_HIT: f32 = 0.7;
pub const PROB_MISS: f32 = 0.4;
pub const CLAMP_MIN: f32 = -2.0;
pub const CLAMP_MAX: f32 = 3.5;
/// Log-odds above which a voxel counts as occupied (p > 0.5).
pub const OCCUPANCY_THRESHOLD: f32 = 0.0;

fn logit(p: f32) -> f32 {
    (p / (1.0 - p)).ln()
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MapStats {
    pub total_voxels: usize,
    pub occupied: usize,
    pub free: usize,
}

/// Sparse probabilistic occupancy grid.
pub struct OccupancyMap {
    resolution: f32,
    inv_resolution: f32,
    log_hit: f32,
    log_miss: f32,
    /// Log-odds per voxel. Absent means "unknown".
    voxels: HashMap<Key, f32>,
    /// Reused between insertions so a scan does not reallocate every frame.
    scratch: Vec<Key>,
}

impl OccupancyMap {
    pub fn new(resolution: f32) -> Self {
        assert!(resolution > 0.0, "resolution must be positive");
        Self {
            resolution,
            inv_resolution: 1.0 / resolution,
            log_hit: logit(PROB_HIT),
            log_miss: logit(PROB_MISS),
            voxels: HashMap::new(),
            scratch: Vec::new(),
        }
    }

    pub fn resolution(&self) -> f32 {
        self.resolution
    }

    /// World point -> voxel key.
    pub fn key_of(&self, p: [f32; 3]) -> Key {
        (
            (p[0] * self.inv_resolution).floor() as i32,
            (p[1] * self.inv_resolution).floor() as i32,
            (p[2] * self.inv_resolution).floor() as i32,
        )
    }

    /// Voxel key -> world centre.
    pub fn centre_of(&self, k: Key) -> [f32; 3] {
        [
            (k.0 as f32 + 0.5) * self.resolution,
            (k.1 as f32 + 0.5) * self.resolution,
            (k.2 as f32 + 0.5) * self.resolution,
        ]
    }

    /// Occupancy probability of a voxel, or `None` if never observed.
    pub fn probability(&self, k: Key) -> Option<f32> {
        self.voxels.get(&k).map(|l| 1.0 - 1.0 / (1.0 + l.exp()))
    }

    pub fn is_occupied(&self, k: Key) -> bool {
        self.voxels
            .get(&k)
            .is_some_and(|&l| l > OCCUPANCY_THRESHOLD)
    }

    fn update(&mut self, k: Key, delta: f32) {
        let entry = self.voxels.entry(k).or_insert(0.0);
        *entry = (*entry + delta).clamp(CLAMP_MIN, CLAMP_MAX);
    }

    /// Insert a scan taken from `origin`.
    ///
    /// With `discretize` the endpoints are collapsed to unique voxels first,
    /// which is what makes repeated insertion cheap: a 640x480 depth frame
    /// puts many pixels in the same voxel, and each duplicate would otherwise
    /// re-trace the same ray.
    ///
    /// `carve_free` traces every ray and marks the voxels it passes through as
    /// free. That is what removes spurious occupied voxels over time; turning
    /// it off makes insertion much cheaper but degenerates into plain point
    /// accumulation.
    pub fn insert_point_cloud(
        &mut self,
        points: &[[f32; 3]],
        origin: [f32; 3],
        discretize: bool,
        carve_free: bool,
    ) {
        let origin_key = self.key_of(origin);

        // Collect endpoint voxels, optionally deduplicated. The buffer is
        // taken out of `self` first so `key_of` can borrow `self` immutably.
        let mut endpoints = std::mem::take(&mut self.scratch);
        endpoints.clear();

        if discretize {
            let mut seen = std::collections::HashSet::<Key>::with_capacity(points.len() / 4 + 1);
            for p in points {
                let k = self.key_of(*p);
                if seen.insert(k) {
                    endpoints.push(k);
                }
            }
        } else {
            endpoints.extend(points.iter().map(|p| self.key_of(*p)));
        }

        if carve_free {
            // Mark free space first, then occupancy, so a voxel that is both
            // an endpoint and on another ray still ends up occupied.
            let mut ray = Vec::new();
            for &end in &endpoints {
                ray.clear();
                trace_ray(origin_key, end, &mut ray);
                for &k in &ray {
                    self.update(k, self.log_miss);
                }
            }
        }

        for &k in &endpoints {
            self.update(k, self.log_hit);
        }

        self.scratch = endpoints;
    }

    /// Every voxel currently believed to be occupied.
    pub fn occupied_voxels(&self) -> impl Iterator<Item = (Key, f32)> + '_ {
        self.voxels
            .iter()
            .filter(|&(_, &l)| l > OCCUPANCY_THRESHOLD)
            .map(|(&k, &l)| (k, l))
    }

    pub fn occupied_centres(&self) -> Vec<[f32; 3]> {
        self.occupied_voxels()
            .map(|(k, _)| self.centre_of(k))
            .collect()
    }

    pub fn stats(&self) -> MapStats {
        let occupied = self
            .voxels
            .values()
            .filter(|&&l| l > OCCUPANCY_THRESHOLD)
            .count();
        MapStats {
            total_voxels: self.voxels.len(),
            occupied,
            free: self.voxels.len() - occupied,
        }
    }
}

/// 3D DDA (Amanatides & Woo) over voxel indices.
///
/// Appends every voxel strictly between `from` and `to` — the endpoint is left
/// out so the caller can mark it occupied without a free update fighting it in
/// the same insertion.
fn trace_ray(from: Key, to: Key, out: &mut Vec<Key>) {
    if from == to {
        return;
    }

    let (mut x, mut y, mut z) = from;
    let (dx, dy, dz) = (to.0 - from.0, to.1 - from.1, to.2 - from.2);

    let (sx, sy, sz) = (dx.signum(), dy.signum(), dz.signum());
    let (ax, ay, az) = (dx.abs(), dy.abs(), dz.abs());

    // Step along the dominant axis, accumulating error for the other two.
    if ax >= ay && ax >= az {
        let mut ey = 2 * ay - ax;
        let mut ez = 2 * az - ax;
        for _ in 0..ax {
            if ey > 0 {
                y += sy;
                ey -= 2 * ax;
            }
            if ez > 0 {
                z += sz;
                ez -= 2 * ax;
            }
            ey += 2 * ay;
            ez += 2 * az;
            x += sx;
            if (x, y, z) == to {
                return;
            }
            out.push((x, y, z));
        }
    } else if ay >= az {
        let mut ex = 2 * ax - ay;
        let mut ez = 2 * az - ay;
        for _ in 0..ay {
            if ex > 0 {
                x += sx;
                ex -= 2 * ay;
            }
            if ez > 0 {
                z += sz;
                ez -= 2 * ay;
            }
            ex += 2 * ax;
            ez += 2 * az;
            y += sy;
            if (x, y, z) == to {
                return;
            }
            out.push((x, y, z));
        }
    } else {
        let mut ex = 2 * ax - az;
        let mut ey = 2 * ay - az;
        for _ in 0..az {
            if ex > 0 {
                x += sx;
                ex -= 2 * az;
            }
            if ey > 0 {
                y += sy;
                ey -= 2 * az;
            }
            ex += 2 * ax;
            ey += 2 * ay;
            z += sz;
            if (x, y, z) == to {
                return;
            }
            out.push((x, y, z));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keys_and_centres_round_trip() {
        let map = OccupancyMap::new(0.1);
        let k = map.key_of([1.234, -0.567, 3.0]);
        assert_eq!(k, (12, -6, 30));

        let c = map.centre_of(k);
        assert!((c[0] - 1.25).abs() < 1e-5, "got {c:?}");
        assert!((c[1] - (-0.55)).abs() < 1e-5, "got {c:?}");
        assert!((c[2] - 3.05).abs() < 1e-5, "got {c:?}");

        // A centre must map back to the key it came from.
        assert_eq!(map.key_of(c), k);
    }

    #[test]
    fn negative_coordinates_floor_correctly() {
        let map = OccupancyMap::new(0.1);
        // Truncation instead of floor would collapse -0.05 and 0.05 together.
        assert_eq!(map.key_of([-0.05, 0.0, 0.0]).0, -1);
        assert_eq!(map.key_of([0.05, 0.0, 0.0]).0, 0);
    }

    #[test]
    fn a_single_hit_becomes_occupied() {
        let mut map = OccupancyMap::new(0.1);
        let p = [1.0, 0.0, 0.0];
        map.insert_point_cloud(&[p], [0.0, 0.0, 0.0], true, false);

        let k = map.key_of(p);
        assert!(map.is_occupied(k));
        assert!(map.probability(k).unwrap() > 0.5);
    }

    #[test]
    fn repeated_misses_clear_a_voxel() {
        let mut map = OccupancyMap::new(0.1);
        let target = [1.0, 0.0, 0.0];
        map.insert_point_cloud(&[target], [0.0, 0.0, 0.0], true, false);
        assert!(map.is_occupied(map.key_of(target)));

        // Now see through it repeatedly: rays ending well beyond must carve it.
        for _ in 0..10 {
            map.insert_point_cloud(&[[2.0, 0.0, 0.0]], [0.0, 0.0, 0.0], true, true);
        }
        assert!(
            !map.is_occupied(map.key_of(target)),
            "free-space carving did not clear the voxel"
        );
    }

    #[test]
    fn log_odds_are_clamped() {
        let mut map = OccupancyMap::new(0.1);
        let p = [1.0, 0.0, 0.0];
        for _ in 0..500 {
            map.insert_point_cloud(&[p], [0.0, 0.0, 0.0], true, false);
        }
        let prob = map.probability(map.key_of(p)).unwrap();
        let max_prob = 1.0 - 1.0 / (1.0 + CLAMP_MAX.exp());
        assert!(
            (prob - max_prob).abs() < 1e-5,
            "prob {prob}, cap {max_prob}"
        );
    }

    #[test]
    fn discretize_collapses_duplicate_endpoints() {
        let mut plain = OccupancyMap::new(0.1);
        let mut discrete = OccupancyMap::new(0.1);

        // Twenty samples inside one voxel.
        let pts: Vec<[f32; 3]> = (0..20)
            .map(|i| [1.0 + i as f32 * 0.001, 0.0, 0.0])
            .collect();

        plain.insert_point_cloud(&pts, [0.0, 0.0, 0.0], false, false);
        discrete.insert_point_cloud(&pts, [0.0, 0.0, 0.0], true, false);

        let k = plain.key_of([1.0, 0.0, 0.0]);
        // Undiscretized hits pile up to the clamp; discretized applies once.
        assert!(plain.probability(k).unwrap() > discrete.probability(k).unwrap());
        assert_eq!(discrete.stats().occupied, 1);
    }

    #[test]
    fn ray_excludes_both_endpoints() {
        let mut ray = Vec::new();
        trace_ray((0, 0, 0), (5, 0, 0), &mut ray);
        assert_eq!(ray, vec![(1, 0, 0), (2, 0, 0), (3, 0, 0), (4, 0, 0)]);
    }

    #[test]
    fn ray_handles_diagonals_and_reversal() {
        let mut ray = Vec::new();
        trace_ray((0, 0, 0), (3, 3, 3), &mut ray);
        assert!(!ray.contains(&(0, 0, 0)));
        assert!(!ray.contains(&(3, 3, 3)));
        assert!(!ray.is_empty());

        // Same span backwards must visit the same number of voxels.
        let mut back = Vec::new();
        trace_ray((3, 3, 3), (0, 0, 0), &mut back);
        assert_eq!(ray.len(), back.len());
    }

    #[test]
    fn identical_endpoints_produce_no_ray() {
        let mut ray = Vec::new();
        trace_ray((2, 2, 2), (2, 2, 2), &mut ray);
        assert!(ray.is_empty());
    }

    #[test]
    fn surface_survives_repeated_observation_with_carving() {
        // The real workload: a wall seen many times from a moving origin must
        // stay occupied, not be erased by its own neighbours' rays.
        let mut map = OccupancyMap::new(0.1);
        let wall: Vec<[f32; 3]> = (0..50).map(|i| [5.0, -2.5 + i as f32 * 0.1, 1.0]).collect();

        for step in 0..20 {
            let origin = [0.0, -1.0 + step as f32 * 0.1, 1.0];
            map.insert_point_cloud(&wall, origin, true, true);
        }

        let still_occupied = wall
            .iter()
            .filter(|p| map.is_occupied(map.key_of(**p)))
            .count();
        assert!(
            still_occupied >= wall.len() - 2,
            "only {still_occupied}/{} wall voxels survived",
            wall.len()
        );
    }
}
