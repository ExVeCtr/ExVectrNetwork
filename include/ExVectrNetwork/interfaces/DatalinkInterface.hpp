#ifndef EXVECTRNETWORK_INTERFACES_DATALINKINTERFACE_HPP_
#define EXVECTRNETWORK_INTERFACES_DATALINKINTERFACE_HPP_

#include "ExVectrCore/handler.hpp"
#include "ExVectrCore/list.hpp"
#include "ExVectrCore/list_buffer.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrHAL/digital_io.hpp"

namespace VCTR::Net {

constexpr size_t dataLinkMaxFrameLength = 250;

struct Dataframe {
  Core::ListBuffer<uint8_t, dataLinkMaxFrameLength> data;
};

/**
 * @brief Interface for the datalink layer. Inhereting classes must implement
 * the dataframeReceiveFunc function.
 * @note The datalink class is responsible for receiving dataframes from the
 * physical layer and sending dataframes to the physical layer.
 */
class Datalink_Interface {
protected:
  /// @brief This handler group is called when a dataframe is received.
  Core::HandlerGroup<const Dataframe &> receiveHandlers_;

public:
  virtual bool transmitDataframe(const Dataframe &dataframe) = 0;

  /**
   * @brief Get the amount of free space in bytes in the transmit buffer.
   * This can be used to check if there is enough space to send a dataframe
   * before attempting to send it.
   *
   * @return size_t The amount of free space in the transmit buffer.
   */
  virtual size_t getBufferFreeSpace() const = 0;

  /**
   * @brief Adds a handler to be called when a dataframe is received.
   * @param handler The handler function to be added.
   */
  void addReceiveHandler(
      Core::HandlerGroup<const Dataframe &>::HandlerFunction handler);

  /**
   * @brief Clears all handlers that are called when a dataframe is received.
   */
  void clearReceiveHandlers();
};

} // namespace VCTR::Net

#endif