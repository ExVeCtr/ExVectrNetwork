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
};

} // namespace VCTR::network::datalink

#endif