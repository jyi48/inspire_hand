# 작업 기록

## 2026-04-09

### inspire_hand repo 생성

- 레퍼런스: https://github.com/NaCl-1374/inspire_hand_ws (clone → d:/GitHub/inspire_hand_ws)
- 기존 Python pymodbus 드라이버 10~12Hz 병목 원인 분석
  - DDS write callback과 Modbus read 루프가 같은 `modbus_lock` 경쟁
  - Python GIL + pymodbus 동기 IO overhead
- C++ libmodbus 기반 드라이버로 재작성 (100Hz 목표)

### inspire_hand_driver (C++ ROS2 패키지)

- `ModbusClient`: libmodbus TCP 래핑 (connect/write/read)
- `InspireHandDriver` 노드:
  - left/right 독립 ModbusClient → 손 간 락 경쟁 없음
  - ctrl callback: `/rt/inspire_hand/ctrl/l|r` subscribe → 즉시 Modbus write
  - 50Hz timer: state read + `/rt/inspire_hand/state/l|r` publish
- IP/port/Hz 전부 ROS2 파라미터로 오버라이드 가능

### 연동 구조

```
[Manus PC] manus_inspire.py
    → ROS2 /rt/inspire_hand/ctrl/l|r (InspireHandCtrl)
        → [RPC] inspire_hand_driver (C++)
            → Modbus TCP → Inspire Hand
```

- manus_inspire.py는 2026 repo에 유지
- inspire_hand_driver는 RPC에서 독립 실행
- 같은 ROS_DOMAIN_ID + LAN 환경 필요
- 기존 inspire_driver.py (unitree DDS 기반)와 동시 실행 금지

### 트러블슈팅 및 아키텍처 개선

**문제 1: pkg-config 없는 환경에서 CMake 빌드 실패**
- RPC가 인터넷 없는 오프라인 환경
- `find_package(PkgConfig QUIET)` + `find_library` fallback으로 해결

**문제 2: Modbus 연결 끊김 (connection timeout / refused)**
- 원인 분석 과정:
  - manus publish 120Hz → ctrl callback 120Hz → Modbus write 120Hz → 손 제어기 과부하
  - write callback과 state read timer가 같은 Modbus 연결 + mutex 경쟁
  - state read 7번 순차 호출 중 write 타임아웃
  - reconnect 시 `modbus_close()` 없이 재연결 시도 → 실패
  - 손 제어기 TCP 연결 1개만 허용 → write/read 연결 분리 불가
- 레지스터 주소 확인: RH56DFTP 모델, 레퍼런스 코드와 전부 일치

**최종 구조 (단일 타이머 아키텍처):**
- `ctrl_callback` → 최신 명령 캐시에 저장만 (Modbus 접근 없음)
- 단일 타이머 100Hz → write → state read → publish 순차 처리
- Modbus 경쟁 구조적으로 제거, mutex 불필요
- state read 7번 개별 read (블록 read 시 빈 레지스터 오염으로 angle_act 값 이상 발생)
- `timer_hz` 파라미터 하나로 write/state 주기 동시 조절 (기본 100Hz)
- reconnect: 항상 `modbus_close()` 먼저 수행

**문제 3: angle_act 값이 datasheet 범위(1000) 초과**
- 연속 블록 read(1534~1599)에서 빈 레지스터(1540~1545 등)가 쓰레기값 반환
- Python 개별 read 방식으로 복귀 → 정상

**ctrl mode 수정 (manus_inspire.py)**
- angle only(0b0001) → angle+force+speed(0b1101)
- speed_set, force_set이 mode bit 꺼져있어서 실제 Modbus에 전달 안 됐던 문제 수정
- force_set = 800 (800g gripping force limit)

**touch 토픽 추가 (미활성화)**
- `/rt/inspire_hand/touch/l|r` 구현 완료, 주석처리 상태
- 레지스터: 3000번대 (fingerone~fingerfive, palm)
- 활성화 시: hpp/cpp 내 `// touch 활성화 시 해제` 주석 전체 해제

**touch read count 수정 (버그 수정, 미활성화 상태)**
- 레퍼런스 Python 분석: data_sheet의 세 번째 값은 바이트 수 (레지스터 수 아님)
  - 레지스터 수 = 바이트 수 / 2 (Modbus 레지스터 1개 = 2바이트)
  - 기존 C++ 코드: 바이트 수를 레지스터 수로 잘못 사용 (18/192/160/224)
  - 올바른 레지스터 수: 9/96/80/112 — 모두 Modbus 한계(125) 이하
- `InspireHandTouch.msg` 수정: `fingerfive_palm_touch` int16[80] → int16[96]
  - SDK IDL(`_inspire_hand_touch.py`) 확인 결과 96이 올바른 값
  - ctrl/state msg는 SDK IDL과 완전 일치 — 문제 없음

**touch 다운샘플링 준비 (미활성화)**
- tick counter 방식으로 touch를 저속(기본 20Hz)으로 실행
  - 100Hz 타이머에서 5번에 1번 실행: ctrl/state 주기 유지하면서 touch 17RTT(~17ms) 허용
  - 4틱 ~6ms, 5번째 틱 ~23ms → 평균 ≈ 9ms
- `touch_hz` 파라미터 (기본 20.0), `touch_interval_` 멤버 — 전부 주석처리 상태
- 활성화 순서: touch_tick, touch_interval_, touch_hz, TouchMsg, touch_pub, do_touch_read_publish 일괄 해제
