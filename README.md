# hw-core

```
hw-core/
  inspire_hand/
    src/
      inspire_hand_driver/
      inspire_hand_msgs/
  rby1_rt/
```

## Dependencies

- ROS2 Humble
- libmodbus: `sudo apt install libmodbus-dev`

## Build

```bash
source /opt/ros/humble/setup.bash

# inspire_hand only
colcon build --base-paths inspire_hand/src

# rby1_rt only
colcon build --base-paths rby1_rt

# all
colcon build --base-paths inspire_hand/src rby1_rt

source install/setup.bash
```

## Run

```bash
ros2 run inspire_hand_driver inspire_hand_driver
```

파라미터 오버라이드:

```bash
ros2 run inspire_hand_driver inspire_hand_driver \
  --ros-args \
  -p right_ip:=<RIGHT_HAND_IP> \
  -p left_ip:=<LEFT_HAND_IP> \
  -p state_hz:=50.0
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `right_ip` | — | 오른손 Modbus TCP IP |
| `left_ip` | — | 왼손 Modbus TCP IP |
| `modbus_port` | `6000` | Modbus TCP 포트 |
| `device_id` | `1` | Modbus device ID |
| `state_hz` | `50.0` | 상태 read/publish 주기 (Hz) |

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/rt/inspire_hand/ctrl/r` | `InspireHandCtrl` | subscribe |
| `/rt/inspire_hand/ctrl/l` | `InspireHandCtrl` | subscribe |
| `/rt/inspire_hand/state/r` | `InspireHandState` | publish |
| `/rt/inspire_hand/state/l` | `InspireHandState` | publish |

---

# rby1_rt

C++ RT node replacing `new_core_main.py` — rby1-sdk → ROS2 bridge with 50Hz RT loop.

## Prerequisites

**1. Build & install gRPC (rby1-sdk dependency)**

apt 버전은 CMake config 파일이 없어 직접 빌드 필요:

```bash
sudo apt install -y build-essential cmake git pkg-config
git clone --recurse-submodules -b v1.54.0 --depth 1 \
  https://github.com/grpc/grpc ~/grpc
cd ~/grpc
cmake -B build \
  -DgRPC_INSTALL=ON \
  -DgRPC_BUILD_TESTS=OFF \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build -j$(nproc)
sudo cmake --install build
sudo ldconfig
```

**2. Build & install rby1-sdk (C++ library)**
```bash
cd ~/rby1-sdk-main
cmake -B build -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build -j$(nproc)
sudo cmake --install build
```

**3. RT priority setup (one-time, requires re-login)**

`/etc/security/limits.conf` 에 추가:
```
nvidia  soft  rtprio  99
nvidia  hard  rtprio  99
```
로그아웃 후 재로그인. 또는 빌드 후 매번:
```bash
sudo setcap cap_sys_nice+ep ~/ros2_ws/install/rby1_rt/lib/rby1_rt/rby1_rt_node
```

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rby1_rt
source install/setup.bash
```

## Run

```bash
ros2 run rby1_rt rby1_rt_node
```

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/rby1_teleop_command` | `JointGroupCommand` | subscribe |
| `/rby1_impedance_teleop_command` | `JointGroupCommand` | subscribe |
| `/rby1_base_velocity` | `Twist` | subscribe |
| `/rby1_status` | `String` (JSON) | publish |
| `/rby1_status_joint` | `JointState` | publish |

Action server: `/rby1_command` (`Rby1Command`)

---

## Author

Jaehyun Yi

## References

- https://github.com/NaCl-1374/inspire_hand_ws
