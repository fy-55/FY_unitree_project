# Third-party notices

The root Apache-2.0 license applies only to original project code and documentation. Third-party material keeps its original license and copyright.

| Material | Location | Upstream | License |
|---|---|---|---|
| Unitree G1 model assets adapted from `unitree_rl_gym` | `src/g1_nav_description` | https://github.com/unitreerobotics/unitree_rl_gym | BSD-3-Clause; see package `LICENSES/` |
| Unitree ROS 2 message definitions | `src/unitree_api`, `src/unitree_go`, `src/unitree_hg` | https://github.com/unitreerobotics/unitree_ros2 | BSD-3-Clause; package license files and copyright remain intact |
| ROS 2, Nav2, SLAM Toolbox and Gazebo | system dependencies | their respective upstream projects | Their respective licenses |

The local `third_party/unitree_slam` copy is excluded from Git because no redistribution license was present in the inspected copy. Users must obtain any official SLAM SDK/example from an authorized Unitree source and follow its terms.
