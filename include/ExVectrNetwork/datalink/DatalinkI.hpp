#ifndef EXVECTRNETWORK_INTERFACES_DATALINKINTERFACE_HPP_
#define EXVECTRNETWORK_INTERFACES_DATALINKINTERFACE_HPP_

#include "ExVectrCore/handler.hpp"

#include "ExVectrNetwork/DataPacket.hpp"

namespace VCTR::network::datalink {

/**
 * @brief Interface for the datalink layer. Inhereting classes must implement
 * the dataframeReceiveFunc function.
 * @note The datalink class is responsible for receiving dataframes from the
 * physical layer and sending dataframes to the physical layer.
 */
class DatalinkI {
protected:
  /// @brief This handler group is called when a dataframe is received.
  Core::HandlerGroup<const DataPacket &> receiveHandlers_;

public:
  virtual bool transmitDataframe(const DataPacket &dataframe) = 0;

  /**
   * @brief Get the maximum packet size that can be transmitted by the datalink.
   * @note packets over this size will be dropped and not transmitted.
   * @return size_t The maximum packet size in bytes.
   */
  virtual size_t getMaxPacketSize() const = 0;

  /**
   * @returns true if the datalink is currently blocked and cannot send
   * dataframes.
   */
  virtual bool isChannelBlocked() const = 0;

  /**
   * @brief Adds a handler to be called when a dataframe is received.
   * @param handler The handler function to be added.
   */
  void addReceiveHandler(
      Core::HandlerGroup<const DataPacket &>::HandlerFunction handler);

  /**
   * @brief Clears all handlers that are called when a dataframe is received.
   */
  void clearReceiveHandlers();
};

} // namespace VCTR::network::datalink

#endif