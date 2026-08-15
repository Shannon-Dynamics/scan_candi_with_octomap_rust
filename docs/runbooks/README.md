# Runbooks

Operational procedures. Every runbook ends with a **Verification** section
naming the number or the file that must appear — not "it should work".

| Runbook | When to use it |
|---|---|
| [`windows-live-scan.md`](windows-live-scan.md) | Demos, quick regression checks, rebuilding the backup recording. No ROS 2 |
| [`wsl-ros2.md`](wsl-ros2.md) | Running the pipeline over DDS, taking the octree/hash-grid measurement, feeding RViz |
| [`asset-conversion.md`](asset-conversion.md) | The source mesh changed, or a new workspace needs setting up |
| [`troubleshooting.md`](troubleshooting.md) | **Read first** when something fails in a strange way |

The format for a new runbook is in [`_template.md`](_template.md).
