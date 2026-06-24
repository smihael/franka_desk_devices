# Custom end-effector packages for Franka robots

These packages provide pre-configured visualization assets for grippers and sensors used with Franka robots, installable via Franka Desk > End Effector Settings > Devices > Install.

## Packages

| File |  Description |
|------|-------------|
| `robotiq-2f85.device` | Robotiq 2F85 gripper |
| `onrobot-ft.device` | OnRobot HEX/e V2 force-torque sensor |
| `combined-sensor-gripper.device` | OnRobot HEX/e + Robotiq 2F85 combined |

## Repacking

After editing files in a device directory:

```bash
tar cvzf output.device -C directory_name .
```

## Key Constraints

- Package must be under 20 MB
- DAE effects must use minimal phong: `emission`, `diffuse`, `index_of_refraction` only
- No Camera or Light nodes in DAEs
- URDF `mesh` paths use `package://franka_description/` prefix
- `endeffector-config.json` transformation is column-major 4x4 matrix

See `SKILL.md` for the full workflow. 

## References

This project is based on community and open-source work:

- https://www.franka-community.de/t/how-do-i-add-end-effector-visualization/8591
- https://github.com/gbartyzel/ros2_net_ft_driver/tree/rolling/net_ft_description
- https://github.com/PickNikRobotics/ros2_robotiq_gripper/tree/main/robotiq_description

*This repository was generated with the assistance of an AI system (MiMo-V2.5).*
