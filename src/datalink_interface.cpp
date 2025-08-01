#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrCore/print.hpp"
#include "ExVectrCore/list.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrHAL/digital_io.hpp"

#include "ExVectrNetwork/interfaces/datalink_interface.hpp"

namespace VCTR
{

    namespace Net
    {

        Datalink_Interface::Datalink_Interface()
        {
            transmitSubr_.setCallback(this, &Datalink_Interface::dataframeReceiveFunc);
        }

        Core::Topic<Core::List<uint8_t>> &Datalink_Interface::getReceiveTopic()
        {
            return receiveTopic_;
        }

        void Datalink_Interface::setTransmitTopic(Core::Topic<Core::List<uint8_t>> &transmitTopic)
        {
            transmitSubr_.subscribe(transmitTopic);
        }

        void Datalink_Interface::removeTransmitTopic()
        {
            transmitSubr_.unsubscribe();
        }

    }

}