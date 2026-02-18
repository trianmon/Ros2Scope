<!-- SPDX-FileCopyrightText: 2026 Georgy Grigoriev (GitHub: trianmon) -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# ROS 2 Scope (r2s)

ROS 2 desktop scope application with Dear ImGui UI and runtime plugin loading.

## Build and Install

```bash
cd /path/to/ros2scope
source /opt/ros/$ROS_DISTRO/setup.bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
cmake --install build --prefix ~/.local
```

Optional system-wide install:

```bash
sudo cmake --install build --prefix /usr/local
```

## Run

Use a terminal with the ROS environment you want to inspect:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
# optional overlay workspace
# source /path/to/ws/install/setup.bash
r2s
```

Common options:

```bash
r2s --layout minimal.json
r2s --plugin topic_echo
r2s --safe-mode
```

## License

Licensed under the Apache License 2.0.
See [LICENSE](LICENSE), [NOTICE](NOTICE), and [AUTHORS](AUTHORS).
