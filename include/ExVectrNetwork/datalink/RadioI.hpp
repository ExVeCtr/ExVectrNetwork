#ifndef EXVECTRNETWORK_INTERFACES_RADIOINTERFACE_HPP_
#define EXVECTRNETWORK_INTERFACES_RADIOINTERFACE_HPP_

#include "ExVectrNetwork/datalink/DatalinkI.hpp"
#include "ExVectrNetwork/physical/HasChannels.hpp"

namespace VCTR::network::datalink {

/**
 * @brief Interface for the a datalink that also has channel control. Inhereting
 * classes must implement the dataframeReceiveFunc function.
 * @note The datalink class is responsible for receiving dataframes from the
 * physical layer and sending dataframes to the physical layer.
 */
class RadioI : public VCTR::network::datalink::DatalinkI,
               public VCTR::network::physical::HasChannels {
public:
  /// @brief SNR of the last successfully received packet (dB).
  /// Override in implementations that can provide RF metrics.
  virtual int16_t lastPacketSNR() const = 0;

  /**
   * @brief Manually put the radio hardware into or out of RX mode.
   * When enabled the radio enters continuous RX (idle-receive).
   * When disabled the radio is placed into standby immediately.
   * This is independent of setEnableTxRx – even if TxRx is disabled the
   * radio will still physically sit in RX, it just won't dispatch packets.
   */
  virtual void setStartReceive(bool rxEnabled) = 0;

  /**
   * @brief Logical TX/RX gate.
   * When disabled:
   *  - The radio still goes through RX phases normally but receive handlers
   *    are NOT called (received data is silently discarded).
   *  - transmitDataframe() discards the packet and returns false.
   *  - isChannelBlocked() always returns true.
   * When enabled, normal operation resumes.
   * This is independent of setStartReceive.
   */
  virtual void setEnableTxRx(bool enable) = 0;

  /**
   * @brief Enable or disable the radio's automatic RX mode after transmission.
   */
  virtual void setEnableAutoRx(bool enableAutoRx) = 0;
};

} // namespace VCTR::network::datalink

#endif