# Repository Layout

This firmware repository is intentionally minimal and only exposes the parts required for deployment.

- `configs/`
  runtime configuration mounted into the container
- `docker/`
  Docker build and runtime entrypoint files
- `scripts/`
  firmware operation entrypoints
- `third_party/`
  vendored third-party dependencies required by the build
- `workspace/src/`
  ROS source packages used by the runtime
- `machine.env`
  airframe-specific configuration surface
- `.env`
  deployment defaults updated automatically by `configure_machine.sh`
