#ifndef EXVECTRNETWORK_HASCHANNELS_HPP_
#define EXVECTRNETWORK_HASCHANNELS_HPP_

#include <stddef.h>

namespace VCTR::network::physical {

/**
 * @brief Defines the interface for controlling a physical layer with channels.
 * @example Frequency Hopping Spread Spectrum (FHSS) with LoRa.
 */
class HasChannels {
public:
  virtual size_t getNumChannels() const = 0;

  virtual size_t getCurrentChannel() const = 0;

  virtual void setCurrentChannel(size_t channel) = 0;
};

} // namespace VCTR::network::physical

#endif // EXVECTRNETWORK_HASCHANNELS_HPP_