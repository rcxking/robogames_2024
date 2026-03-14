# robomagellan_controller

This package contains the configuration and launch files used to spawn the stock
`ros2_controls` differential drive controller.

## config files
`robomagellan_controllers.yaml` contains the `ros2_control` settings for the
differential drive controller.  Once the speed/acceleration limits are
determined change them here.

## launch files
`controller.launch.py` loads the differential drive controller using the
`controller_manager`.

## Changelog
`0.1.0` - Initial package
