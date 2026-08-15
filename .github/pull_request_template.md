## What changed

<!-- One paragraph. What does this do that the repo did not do before? -->

## Why

<!-- Link the ADR if this changes a design decision, the issue otherwise. -->

## Measurements

<!-- This project is measured, not asserted. If you touched the pipeline,
     paste the before/after numbers. The occupied-voxel count is the one that
     must not move for an unrelated reason. -->

| Quantity | Before | After |
|---|---:|---:|
| Occupied voxels @ 0.1 m | 56,063 (`live_scan`) / 56,065 (ROS 2) | |
| Insertion per frame | 0.4 ms | |
| Points per frame | ~9,100 | |

## Checklist

- [ ] `cargo fmt --all -- --check` is clean
- [ ] `cargo clippy --workspace --all-targets -- -D warnings` is clean
- [ ] `cargo test` passes (45 unit tests at last count)
- [ ] `cargo build --release` succeeds
- [ ] If the ROS 2 path changed: `./ros2/run_demo.sh` completes and the mapper's
      comparison table still shows both maps agreeing
- [ ] No `octomap-core` change is needed — or if it is, it has its own pull
      request in the `octo_map_rust` repository
- [ ] New design decision recorded as an ADR in `docs/decisions/`
- [ ] `CHANGELOG.md` updated under `## [Unreleased]`
- [ ] Measured numbers in `docs/05-results.md` updated if they moved
