# qcar2_mixer

ROS 2 Humble package scaffold for the final QCar2 command mixer (mux/mixer stage before hardware).

## Scope in this commit

- Package structure only.
- No node implementation yet.
- Base launch and config placeholders added.
- Dependencies aligned with QCar2 project packages and interfaces.

## Intended role

`qcar2_mixer` will be the final stage that receives command inputs (MotorCommands) from upstream packages and publishes final output to:

- `/qcar2_motor_speed_cmd` (`qcar2_interfaces/msg/MotorCommands`)

This topic is consumed by QCar2 hardware-side components.

## Included placeholders

- Input motor topics parameter list
- Output motor topic parameter
- Lidar topic parameter
- Object detection topics from `qcar2_object_detections`

## Build

From workspace root:

```bash
colcon build --packages-select qcar2_mixer
```
