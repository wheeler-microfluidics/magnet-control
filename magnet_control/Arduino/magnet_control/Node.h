#ifndef ___NODE__H___
#define ___NODE__H___

#include <stdint.h>
#include <Arduino.h>
#include <Servo.h>
#include <NadaMQ.h>
#include <BaseNodeRpc.h>
#include <BaseNodeEeprom.h>
#include <BaseNodeI2c.h>
#include <BaseNodeConfig.h>
#include <BaseNodeSerialHandler.h>
#include <BaseNodeI2cHandler.h>
#include <Array.h>
#include <I2cHandler.h>
#include <SerialHandler.h>
#include <pb_validate.h>
#include <pb_eeprom.h>
#include "magnet_control_config_validate.h"
#include "ServoControl/config_pb.h"


namespace magnet_control {
const size_t FRAME_SIZE = (3 * sizeof(uint8_t)  // Frame boundary
                           - sizeof(uint16_t)  // UUID
                           - sizeof(uint16_t)  // Payload length
                           - sizeof(uint16_t));  // CRC

class Node;

typedef nanopb::EepromMessage<magnet_control_Config,
                              config_validate::Validator<Node> > config_t;


class Node :
  public BaseNode,
  public BaseNodeEeprom,
  public BaseNodeI2c,
  public BaseNodeConfig<config_t>,
#ifndef DISABLE_SERIAL
  public BaseNodeSerialHandler,
#endif  // #ifndef DISABLE_SERIAL
  public BaseNodeI2cHandler<base_node_rpc::i2c_handler_t> {
public:
  typedef PacketParser<FixedPacket> parser_t;
  static const uint16_t BUFFER_SIZE = 128;  // >= longest property string

  uint8_t buffer_[BUFFER_SIZE];
  uint32_t tick_count_;
  int32_t target_position_;
  Servo servo_;

  Node() : BaseNode(), BaseNodeConfig<config_t>(magnet_control_Config_fields) {}

  UInt8Array get_buffer() { return UInt8Array(sizeof(buffer_), buffer_); }
  /* This is a required method to provide a temporary buffer to the
   * `BaseNode...` classes. */

  void begin();
  void set_i2c_address(uint8_t value);  // Override to validate i2c address

  uint8_t servo_read() { return servo_.read(); }
  void servo_write(uint8_t angle) { servo_.write(angle); }
  void servo_write_microseconds(uint16_t us) { servo_.writeMicroseconds(us); }
  bool servo_attached() { return servo_.attached(); }
  void servo_detach() { servo_.detach(); }
  void servo_attach(uint8_t servo_pin) { servo_.attach(servo_pin); }
};


}  // namespace magnet_control


#endif  // #ifndef ___NODE__H___
