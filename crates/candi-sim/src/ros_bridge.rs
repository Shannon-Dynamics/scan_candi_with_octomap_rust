//! Publish the scan as ROS 2 topics.
//!
//! Message construction is kept out of the transport layer: [`CloudFrame`]
//! and [`Transform`] are plain data, and the node binary turns them into
//! `sensor_msgs/PointCloud2` and `tf2_msgs/TFMessage`. That keeps the
//! throttling and byte layout unit-testable without a ROS installation.

use std::time::{Duration, Instant};

/// Frame ids used across the pipeline.
pub const FRAME_MAP: &str = "map";
pub const FRAME_DRONE: &str = "drone";

/// Publish rate the pipeline is budgeted against.
pub const PUBLISH_HZ: f64 = 10.0;

/// One cloud ready to be wrapped in a PointCloud2.
#[derive(Debug, Clone)]
pub struct CloudFrame {
    /// Tightly packed little-endian f32 xyz triples.
    pub data: Vec<u8>,
    pub point_count: u32,
}

impl CloudFrame {
    pub const POINT_STEP: u32 = 12;

    pub fn from_points(points: &[[f32; 3]]) -> Self {
        let mut data = Vec::with_capacity(points.len() * Self::POINT_STEP as usize);
        for p in points {
            for v in p {
                data.extend_from_slice(&v.to_le_bytes());
            }
        }
        Self {
            data,
            point_count: points.len() as u32,
        }
    }

    pub fn row_step(&self) -> u32 {
        Self::POINT_STEP * self.point_count
    }
}

/// Drone pose for a `geometry_msgs/TransformStamped`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform {
    pub translation: [f64; 3],
    /// MuJoCo order [w, x, y, z]; ROS wants [x, y, z, w], so use
    /// [`Transform::quat_xyzw`] when filling the message.
    pub quat_wxyz: [f64; 4],
}

impl Transform {
    pub fn quat_xyzw(&self) -> [f64; 4] {
        let [w, x, y, z] = self.quat_wxyz;
        [x, y, z, w]
    }
}

/// Fixed-rate gate for the publishers.
///
/// The simulation loop steps at hundreds of Hz, well above the 10 Hz the
/// pipeline publishes at, so without this gate it would flood the topics.
pub struct RateLimiter {
    interval: Duration,
    last: Option<Instant>,
}

impl RateLimiter {
    pub fn new(hz: f64) -> Self {
        assert!(hz > 0.0, "rate must be positive");
        Self {
            interval: Duration::from_secs_f64(1.0 / hz),
            last: None,
        }
    }

    /// True when the interval has elapsed, recording the tick if so.
    pub fn should_publish(&mut self) -> bool {
        self.should_publish_at(Instant::now())
    }

    /// Testable form: the caller supplies the clock reading.
    pub fn should_publish_at(&mut self, now: Instant) -> bool {
        match self.last {
            Some(prev) if now.duration_since(prev) < self.interval => false,
            _ => {
                self.last = Some(now);
                true
            }
        }
    }

    pub fn interval(&self) -> Duration {
        self.interval
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cloud_frame_packs_twelve_bytes_per_point() {
        let frame = CloudFrame::from_points(&[[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
        assert_eq!(frame.point_count, 2);
        assert_eq!(frame.data.len(), 24);
        assert_eq!(frame.row_step(), 24);

        // First point must decode back exactly.
        assert_eq!(
            f32::from_le_bytes(frame.data[0..4].try_into().unwrap()),
            1.0
        );
        assert_eq!(
            f32::from_le_bytes(frame.data[8..12].try_into().unwrap()),
            3.0
        );
    }

    #[test]
    fn empty_cloud_is_valid() {
        let frame = CloudFrame::from_points(&[]);
        assert_eq!(frame.point_count, 0);
        assert!(frame.data.is_empty());
        assert_eq!(frame.row_step(), 0);
    }

    // 0.7071 is a rounded literal standing in for a 90° rotation, not an
    // attempt to spell 1/sqrt(2): the test asserts that the reorder passes the
    // components through untouched, so the exact value is irrelevant and
    // substituting the constant would only make the assertion harder to read.
    #[allow(clippy::approx_constant)]
    #[test]
    fn quaternion_is_reordered_for_ros() {
        let t = Transform {
            translation: [1.0, 2.0, 3.0],
            quat_wxyz: [0.7071, 0.0, 0.0, 0.7071],
        };
        // ROS puts w last.
        assert_eq!(t.quat_xyzw(), [0.0, 0.0, 0.7071, 0.7071]);
    }

    #[test]
    fn limiter_passes_the_first_call_then_throttles() {
        let mut limiter = RateLimiter::new(PUBLISH_HZ);
        let t0 = Instant::now();

        assert!(limiter.should_publish_at(t0));
        assert!(!limiter.should_publish_at(t0 + Duration::from_millis(50)));
        assert!(!limiter.should_publish_at(t0 + Duration::from_millis(99)));
        assert!(limiter.should_publish_at(t0 + Duration::from_millis(100)));
    }

    #[test]
    fn limiter_yields_the_requested_rate() {
        let mut limiter = RateLimiter::new(PUBLISH_HZ);
        let t0 = Instant::now();

        // One simulated second, stepped at 1 ms — the sim's real cadence.
        let published = (0..1000)
            .filter(|i| limiter.should_publish_at(t0 + Duration::from_millis(*i)))
            .count();
        assert_eq!(published, 10, "expected 10 Hz, got {published}");
    }

    #[test]
    fn interval_matches_the_rate() {
        assert_eq!(
            RateLimiter::new(10.0).interval(),
            Duration::from_millis(100)
        );
    }
}
