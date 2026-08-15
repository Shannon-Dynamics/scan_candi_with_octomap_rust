//! Height gradient for occupied voxels.
//!
//! blue (low) -> cyan -> green -> yellow -> red (high), shared by the ROS
//! MarkerArray and the rerun points so the two views always agree.

/// Gradient stops, evenly spaced from low to high.
const STOPS: [[f32; 3]; 5] = [
    [0.10, 0.25, 0.90], // blue
    [0.10, 0.80, 0.90], // cyan
    [0.15, 0.80, 0.25], // green
    [0.95, 0.85, 0.15], // yellow
    [0.90, 0.20, 0.15], // red
];

/// Colour for a normalized height in `0.0..=1.0`, as linear RGB floats.
pub fn height_colour_f32(t: f32) -> [f32; 3] {
    let t = t.clamp(0.0, 1.0);
    let span = (STOPS.len() - 1) as f32;
    let scaled = t * span;
    let i = (scaled.floor() as usize).min(STOPS.len() - 2);
    let frac = scaled - i as f32;

    let (a, b) = (STOPS[i], STOPS[i + 1]);
    [
        a[0] + (b[0] - a[0]) * frac,
        a[1] + (b[1] - a[1]) * frac,
        a[2] + (b[2] - a[2]) * frac,
    ]
}

/// Same gradient as 8-bit RGB.
pub fn height_colour_u8(t: f32) -> [u8; 3] {
    let c = height_colour_f32(t);
    [
        (c[0] * 255.0).round().clamp(0.0, 255.0) as u8,
        (c[1] * 255.0).round().clamp(0.0, 255.0) as u8,
        (c[2] * 255.0).round().clamp(0.0, 255.0) as u8,
    ]
}

/// Normalize `z` against a height range, guarding a zero-height span.
pub fn normalize(z: f32, z_min: f32, z_max: f32) -> f32 {
    if (z_max - z_min).abs() < 1e-6 {
        0.5
    } else {
        (z - z_min) / (z_max - z_min)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The top stop is reached by interpolating with frac = 1.0, so it lands
    /// within rounding of the stop rather than bit-identical to it.
    fn assert_close(got: [f32; 3], want: [f32; 3]) {
        for a in 0..3 {
            assert!(
                (got[a] - want[a]).abs() < 1e-6,
                "got {got:?}, want {want:?}"
            );
        }
    }

    #[test]
    fn endpoints_are_blue_and_red() {
        assert_close(height_colour_f32(0.0), STOPS[0]);
        assert_close(height_colour_f32(1.0), STOPS[4]);
    }

    #[test]
    fn out_of_range_input_is_clamped() {
        assert_close(height_colour_f32(-5.0), STOPS[0]);
        assert_close(height_colour_f32(9.0), STOPS[4]);
    }

    #[test]
    fn midpoint_lands_on_the_green_stop() {
        let c = height_colour_f32(0.5);
        for a in 0..3 {
            assert!((c[a] - STOPS[2][a]).abs() < 1e-5, "got {c:?}");
        }
    }

    #[test]
    fn gradient_is_continuous() {
        // No visible banding: consecutive samples stay close together.
        let mut prev = height_colour_f32(0.0);
        for i in 1..=100 {
            let c = height_colour_f32(i as f32 / 100.0);
            for a in 0..3 {
                assert!(
                    (c[a] - prev[a]).abs() < 0.1,
                    "jump at {i}: {prev:?} -> {c:?}"
                );
            }
            prev = c;
        }
    }

    #[test]
    fn blue_gives_way_to_red_with_height() {
        let low = height_colour_f32(0.0);
        let high = height_colour_f32(1.0);
        assert!(low[2] > low[0], "low end should be blue-dominant");
        assert!(high[0] > high[2], "high end should be red-dominant");
    }

    #[test]
    fn normalize_handles_a_flat_range() {
        assert_eq!(normalize(3.0, 3.0, 3.0), 0.5);
        assert!((normalize(3.0, 0.0, 6.0) - 0.5).abs() < 1e-6);
    }

    #[test]
    fn u8_matches_the_float_gradient() {
        let c = height_colour_u8(0.0);
        assert_eq!(c, [26, 64, 230]);
    }
}
