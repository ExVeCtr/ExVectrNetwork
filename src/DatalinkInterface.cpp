#include "ExVectrCore/list.hpp"
#include "ExVectrCore/print.hpp"
#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrHAL/digital_io.hpp"

#include "ExVectrNetwork/interfaces/DatalinkInterface.hpp"

namespace VCTR::Net {

void Datalink_Interface::addReceiveHandler(
    Core::HandlerGroup<const Dataframe &>::HandlerFunction handler) {
  receiveHandlers_.addHandler(handler);
}

void Datalink_Interface::clearReceiveHandlers() {
  receiveHandlers_.clearHandlers();
}

} // namespace VCTR::Net