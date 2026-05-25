#ifndef EXVECTRNETWORK_DATALINK_SX1280_SETTINGS_HPP_
#define EXVECTRNETWORK_DATALINK_SX1280_SETTINGS_HPP_

#include <stdint.h>

#include "sx12xxAL/src/SX128XLT_Definitions.h"

namespace VCTR::network::datalink {

enum SX1280_SF : uint8_t {
  SF_5 = LORA_SF5,
  SF_6 = LORA_SF6,
  SF_7 = LORA_SF7,
  SF_8 = LORA_SF8,
  SF_9 = LORA_SF9,
  SF_10 = LORA_SF10,
  SF_11 = LORA_SF11,
  SF_12 = LORA_SF12
};

enum SX1280_BW : uint8_t {
  BW_200KHz = LORA_BW_0200,
  BW_400KHz = LORA_BW_0400,
  BW_800KHz = LORA_BW_0800,
  BW_1600KHz = LORA_BW_1600,
};

enum SX1280_CR : uint8_t {
  CR_4_5 = LORA_CR_4_5,
  CR_4_6 = LORA_CR_4_6,
  CR_4_7 = LORA_CR_4_7,
  CR_4_8 = LORA_CR_4_8,
  LI_4_5 = LORA_CR_LI_4_5,
  LI_4_6 = LORA_CR_LI_4_6,
  LI_4_8 = LORA_CR_LI_4_8
};

/**
 * @brief Packet framing mode for the SX1280 driver.
 *
 * Dynamic:  Explicit LoRa header – variable length on-air.  Default.
 * Limited:  Implicit LoRa header – fixed OTA size.  A 1-byte length prefix
 *           is prepended so the receiver knows the actual payload length.
 *           OTA size = fixedPacketLength + 1.  User payload capacity =
 *           fixedPacketLength.
 * Fixed:    Implicit LoRa header – fixed OTA size, no length prefix.
 *           User payload must always be exactly fixedPacketLength bytes.
 */
enum class SX1280_PacketMode : uint8_t { Dynamic, Limited, Fixed };

} // namespace VCTR::network::datalink

#endif