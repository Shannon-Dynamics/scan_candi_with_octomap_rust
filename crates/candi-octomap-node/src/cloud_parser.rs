//! Decode `sensor_msgs/PointCloud2` into plain points.
//!
//! Kept free of any ROS types so it can be unit-tested without a ROS
//! installation: the node passes the message's fields and byte buffer in, and
//! the same code runs whether the bytes came off a topic or a test fixture.

/// Numeric type codes from `sensor_msgs/PointField`.
pub const INT8: u8 = 1;
pub const UINT8: u8 = 2;
pub const INT16: u8 = 3;
pub const UINT16: u8 = 4;
pub const INT32: u8 = 5;
pub const UINT32: u8 = 6;
pub const FLOAT32: u8 = 7;
pub const FLOAT64: u8 = 8;

/// One entry of `PointCloud2::fields`.
#[derive(Debug, Clone, PartialEq)]
pub struct Field {
    pub name: String,
    pub offset: u32,
    pub datatype: u8,
    pub count: u32,
}

impl Field {
    pub fn new(name: &str, offset: u32, datatype: u8) -> Self {
        Self {
            name: name.to_string(),
            offset,
            datatype,
            count: 1,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ParseError {
    MissingField(&'static str),
    /// x, y and z must all be FLOAT32 or all FLOAT64.
    UnsupportedDatatype(u8),
    MixedDatatypes,
    ZeroPointStep,
    /// A point would read past the end of `data`.
    Truncated {
        needed: usize,
        available: usize,
    },
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::MissingField(n) => write!(f, "PointCloud2 has no '{n}' field"),
            Self::UnsupportedDatatype(d) => write!(f, "unsupported datatype {d} for x/y/z"),
            Self::MixedDatatypes => write!(f, "x, y and z must share one datatype"),
            Self::ZeroPointStep => write!(f, "point_step is zero"),
            Self::Truncated { needed, available } => {
                write!(f, "buffer holds {available} bytes, need {needed}")
            }
        }
    }
}

impl std::error::Error for ParseError {}

/// Extract xyz points from a PointCloud2 payload.
///
/// Offsets come from `fields` rather than being assumed, so a cloud carrying
/// extra channels (intensity, rgb, padding) parses correctly. Values are read
/// little-endian; `is_bigendian` clouds are vanishingly rare in ROS 2 and are
/// not supported rather than silently mis-decoded.
pub fn parse_pointcloud2(
    data: &[u8],
    fields: &[Field],
    point_step: u32,
    width: u32,
    height: u32,
) -> Result<Vec<[f32; 3]>, ParseError> {
    if point_step == 0 {
        return Err(ParseError::ZeroPointStep);
    }

    let find = |name: &'static str| {
        fields
            .iter()
            .find(|f| f.name == name)
            .ok_or(ParseError::MissingField(name))
    };
    let (fx, fy, fz) = (find("x")?, find("y")?, find("z")?);

    if fx.datatype != fy.datatype || fx.datatype != fz.datatype {
        return Err(ParseError::MixedDatatypes);
    }
    let width_bytes = match fx.datatype {
        FLOAT32 => 4usize,
        FLOAT64 => 8usize,
        other => return Err(ParseError::UnsupportedDatatype(other)),
    };

    let count = (width as usize) * (height as usize);
    let step = point_step as usize;

    let needed = count.saturating_sub(1) * step
        + (fx.offset.max(fy.offset).max(fz.offset) as usize + width_bytes);
    if count > 0 && needed > data.len() {
        return Err(ParseError::Truncated {
            needed,
            available: data.len(),
        });
    }

    let read = |base: usize, offset: u32| -> f32 {
        let at = base + offset as usize;
        match width_bytes {
            4 => f32::from_le_bytes(data[at..at + 4].try_into().unwrap()),
            _ => f64::from_le_bytes(data[at..at + 8].try_into().unwrap()) as f32,
        }
    };

    let mut points = Vec::with_capacity(count);
    for i in 0..count {
        let base = i * step;
        let p = [
            read(base, fx.offset),
            read(base, fy.offset),
            read(base, fz.offset),
        ];
        // Unordered clouds pad with NaN to mark invalid returns.
        if p.iter().all(|v| v.is_finite()) {
            points.push(p);
        }
    }

    Ok(points)
}

/// Build the xyz field layout this project publishes: three FLOAT32s, tightly
/// packed, `point_step` 12.
pub fn xyz_fields() -> Vec<Field> {
    vec![
        Field::new("x", 0, FLOAT32),
        Field::new("y", 4, FLOAT32),
        Field::new("z", 8, FLOAT32),
    ]
}

/// Serialize points into a PointCloud2 payload matching [`xyz_fields`].
pub fn encode_xyz(points: &[[f32; 3]]) -> Vec<u8> {
    let mut data = Vec::with_capacity(points.len() * 12);
    for p in points {
        for v in p {
            data.extend_from_slice(&v.to_le_bytes());
        }
    }
    data
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_three_known_points() {
        let points = [[1.0f32, 2.0, 3.0], [-4.5, 0.25, 100.0], [0.0, 0.0, 0.0]];
        let data = encode_xyz(&points);

        let parsed = parse_pointcloud2(&data, &xyz_fields(), 12, 3, 1).unwrap();
        assert_eq!(parsed, points);
    }

    #[test]
    fn honours_field_offsets_and_padding() {
        // A 32-byte stride with xyz at offset 16, mimicking a cloud that also
        // carries intensity and padding.
        let fields = vec![
            Field::new("intensity", 0, FLOAT32),
            Field::new("x", 16, FLOAT32),
            Field::new("y", 20, FLOAT32),
            Field::new("z", 24, FLOAT32),
        ];
        let mut data = vec![0u8; 64];
        for (i, p) in [[1.0f32, 2.0, 3.0], [4.0, 5.0, 6.0]].iter().enumerate() {
            let base = i * 32;
            data[base + 16..base + 20].copy_from_slice(&p[0].to_le_bytes());
            data[base + 20..base + 24].copy_from_slice(&p[1].to_le_bytes());
            data[base + 24..base + 28].copy_from_slice(&p[2].to_le_bytes());
        }

        let parsed = parse_pointcloud2(&data, &fields, 32, 2, 1).unwrap();
        assert_eq!(parsed, vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
    }

    #[test]
    fn reads_float64_clouds() {
        let fields = vec![
            Field::new("x", 0, FLOAT64),
            Field::new("y", 8, FLOAT64),
            Field::new("z", 16, FLOAT64),
        ];
        let mut data = Vec::new();
        for v in [1.5f64, -2.5, 3.5] {
            data.extend_from_slice(&v.to_le_bytes());
        }

        let parsed = parse_pointcloud2(&data, &fields, 24, 1, 1).unwrap();
        assert_eq!(parsed, vec![[1.5, -2.5, 3.5]]);
    }

    #[test]
    fn ordered_clouds_use_width_times_height() {
        let points: Vec<[f32; 3]> = (0..6).map(|i| [i as f32, 0.0, 0.0]).collect();
        let data = encode_xyz(&points);

        let parsed = parse_pointcloud2(&data, &xyz_fields(), 12, 3, 2).unwrap();
        assert_eq!(parsed.len(), 6);
    }

    #[test]
    fn non_finite_points_are_dropped() {
        let points = [
            [1.0f32, 2.0, 3.0],
            [f32::NAN, 0.0, 0.0],
            [0.0, f32::INFINITY, 0.0],
            [4.0, 5.0, 6.0],
        ];
        let data = encode_xyz(&points);

        let parsed = parse_pointcloud2(&data, &xyz_fields(), 12, 4, 1).unwrap();
        assert_eq!(parsed, vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
    }

    #[test]
    fn missing_field_is_reported() {
        let fields = vec![Field::new("x", 0, FLOAT32), Field::new("y", 4, FLOAT32)];
        let err = parse_pointcloud2(&[0; 12], &fields, 12, 1, 1).unwrap_err();
        assert_eq!(err, ParseError::MissingField("z"));
    }

    #[test]
    fn truncated_buffer_is_reported_not_panicked() {
        // Claims four points but only carries two.
        let data = encode_xyz(&[[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
        let err = parse_pointcloud2(&data, &xyz_fields(), 12, 4, 1).unwrap_err();
        assert!(matches!(err, ParseError::Truncated { .. }), "got {err:?}");
    }

    #[test]
    fn mixed_and_unsupported_datatypes_are_rejected() {
        let mixed = vec![
            Field::new("x", 0, FLOAT32),
            Field::new("y", 4, FLOAT64),
            Field::new("z", 12, FLOAT32),
        ];
        assert_eq!(
            parse_pointcloud2(&[0; 32], &mixed, 32, 1, 1).unwrap_err(),
            ParseError::MixedDatatypes
        );

        let ints = vec![
            Field::new("x", 0, INT32),
            Field::new("y", 4, INT32),
            Field::new("z", 8, INT32),
        ];
        assert_eq!(
            parse_pointcloud2(&[0; 12], &ints, 12, 1, 1).unwrap_err(),
            ParseError::UnsupportedDatatype(INT32)
        );
    }

    #[test]
    fn empty_cloud_parses_to_nothing() {
        assert!(
            parse_pointcloud2(&[], &xyz_fields(), 12, 0, 1)
                .unwrap()
                .is_empty()
        );
        assert_eq!(
            parse_pointcloud2(&[], &xyz_fields(), 0, 1, 1).unwrap_err(),
            ParseError::ZeroPointStep
        );
    }

    #[test]
    fn round_trips_a_realistic_frame() {
        // Roughly one depth frame's worth.
        let points: Vec<[f32; 3]> = (0..9000)
            .map(|i| {
                let t = i as f32 * 0.001;
                [t.sin() * 7.0, t.cos() * 7.0, (i % 60) as f32 * 0.1]
            })
            .collect();
        let data = encode_xyz(&points);
        assert_eq!(data.len(), 9000 * 12);

        let parsed = parse_pointcloud2(&data, &xyz_fields(), 12, points.len() as u32, 1).unwrap();
        assert_eq!(parsed.len(), points.len());
        assert_eq!(parsed[4711], points[4711]);
    }
}
