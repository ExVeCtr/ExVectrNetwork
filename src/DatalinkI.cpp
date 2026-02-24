#include "ExVectrCore/list.hpp"
#include "ExVectrCore/print.hpp"
#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrHAL/digital_io.hpp"

#include "ExVectrNetwork/datalink/DatalinkI.hpp"

namespace VCTR::network::datalink {

void DatalinkI::addReceiveHandler(
    Core::HandlerGroup<const DataPacket &>::HandlerFunction handler) {
  receiveHandlers_.addHandler(handler);
}

void DatalinkI::clearReceiveHandlers() { receiveHandlers_.clearHandlers(); }

} // namespace VCTR::network::datalink