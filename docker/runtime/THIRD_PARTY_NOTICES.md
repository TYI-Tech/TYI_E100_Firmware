## GeographicLib geoid data

`egm96-5.pgm` is bundled in this directory so a clean firmware build does not
depend on live downloads from SourceForge.

The file is the GeographicLib `egm96-5` geoid dataset used by MAVROS during
runtime and was copied from a validated runtime image built for this firmware.

## Vendored projects

- [FAST-LIO](https://github.com/hku-mars/FAST_LIO)
- [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)
- [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)
- [MAVLink](https://github.com/mavlink/mavlink)
- [MAVROS](https://github.com/mavlink/mavros)

The vendored licenses are in the corresponding source directories.
