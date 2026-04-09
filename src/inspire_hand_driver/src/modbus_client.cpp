#include "inspire_hand_driver/modbus_client.hpp"

#include <cstring>
#include <stdexcept>

namespace inspire_hand_driver
{

ModbusClient::ModbusClient(const std::string & ip, int port, int device_id)
: ip_(ip), port_(port), device_id_(device_id)
{
  ctx_ = modbus_new_tcp(ip_.c_str(), port_);
  if (!ctx_) {
    throw std::runtime_error("modbus_new_tcp failed for " + ip_);
  }
  modbus_set_slave(ctx_, device_id_);

  // 응답 타임아웃 10ms
  modbus_set_response_timeout(ctx_, 0, 10000);
}

ModbusClient::~ModbusClient()
{
  disconnect();
  if (ctx_) {
    modbus_free(ctx_);
  }
}

void ModbusClient::connect()
{
  // connected 여부와 무관하게 항상 close 먼저 (no-op if already closed)
  modbus_close(ctx_);
  connected_ = false;

  if (modbus_connect(ctx_) == -1) {
    throw std::runtime_error(
      "Modbus connect failed (" + ip_ + "): " + modbus_strerror(errno));
  }
  connected_ = true;
}

void ModbusClient::disconnect()
{
  if (connected_) {
    modbus_close(ctx_);
    connected_ = false;
  }
}

void ModbusClient::write_registers(int addr, const std::vector<int16_t> & values)
{
  std::vector<uint16_t> regs(values.size());
  std::memcpy(regs.data(), values.data(), values.size() * sizeof(int16_t));

  int rc = modbus_write_registers(ctx_, addr, static_cast<int>(regs.size()), regs.data());
  if (rc == -1) {
    connected_ = false;
    throw std::runtime_error(
      std::string("modbus_write_registers failed: ") + modbus_strerror(errno));
  }
}

std::vector<int16_t> ModbusClient::read_registers_short(int addr, int count)
{
  std::vector<uint16_t> raw(count);
  int rc = modbus_read_registers(ctx_, addr, count, raw.data());
  if (rc == -1) {
    connected_ = false;
    throw std::runtime_error(
      std::string("modbus_read_registers failed: ") + modbus_strerror(errno));
  }

  std::vector<int16_t> result(count);
  for (int i = 0; i < count; ++i) {
    result[i] = static_cast<int16_t>(raw[i]);
  }
  return result;
}

std::vector<uint8_t> ModbusClient::read_registers_byte(int addr, int count)
{
  std::vector<uint16_t> raw(count);
  int rc = modbus_read_registers(ctx_, addr, count, raw.data());
  if (rc == -1) {
    connected_ = false;
    throw std::runtime_error(
      std::string("modbus_read_registers failed: ") + modbus_strerror(errno));
  }

  // 레지스터당 high byte, low byte 분리
  std::vector<uint8_t> result;
  result.reserve(count * 2);
  for (auto reg : raw) {
    result.push_back(static_cast<uint8_t>((reg >> 8) & 0xFF));
    result.push_back(static_cast<uint8_t>(reg & 0xFF));
  }
  return result;
}

}  // namespace inspire_hand_driver
