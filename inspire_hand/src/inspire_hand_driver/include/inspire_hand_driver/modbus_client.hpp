#pragma once

#include <modbus/modbus.h>
#include <string>
#include <vector>
#include <stdexcept>

namespace inspire_hand_driver
{

class ModbusClient
{
public:
  explicit ModbusClient(const std::string & ip, int port = 6000, int device_id = 1);
  ~ModbusClient();

  // 연결/해제
  void connect();
  void disconnect();
  bool is_connected() const { return connected_; }

  // 제어 레지스터 write
  // addr: 시작 레지스터 주소, values: 6개 int16 값
  void write_registers(int addr, const std::vector<int16_t> & values);

  // 상태 레지스터 read (short)
  std::vector<int16_t> read_registers_short(int addr, int count);

  // 상태 레지스터 read (byte — 레지스터당 2 byte)
  std::vector<uint8_t> read_registers_byte(int addr, int count);

private:
  std::string ip_;
  int port_;
  int device_id_;
  modbus_t * ctx_ = nullptr;
  bool connected_ = false;
};

}  // namespace inspire_hand_driver
