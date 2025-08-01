#include "ExVectrCore/print.hpp"
#include "ExVectrCore/list_buffer.hpp"
#include "ExVectrCore/list_extern.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrHAL/digital_io.hpp"

#include "ExVectrNetwork/physicallayers/topic_io.hpp"

namespace VCTR
{

    namespace Net
    {

        TopicIO::TopicIO()
        {
            receiveSubr_.setCallback(this, &TopicIO::receiveItem);
        }

        TopicIO::TopicIO(Core::Topic<const Core::List<uint8_t> &> &topic) : TopicIO()
        {
            receiveSubr_.subscribe(topic);
        }

        void TopicIO::receiveItem(const Core::List<uint8_t> &item)
        {

            if (item.size() > receiveBuffer_.sizeMax() - receiveBuffer_.size())
            {
                LOG_MSG("Buffer overflow. Failure!\n");
                return; // Buffer overflow case. Failure.
            }

            VRBS_MSG("Received %d bytes from topic to send This: %d.\n", item.size(), this);

            for (uint8_t i = 0; i < item.size(); i++)
            {
                receiveBuffer_.placeBack(item[i]);
            }
        }

        void TopicIO::setTopicIO(Core::Topic<const Core::List<uint8_t> &> &topic)
        {
            receiveSubr_.subscribe(topic);
        }

        /*void TopicIO::disconnect(Core::Topic<const uint8_t*>& topic)
        {
            receiveSubr_.unsubscribe(topic);
        }*/

        void TopicIO::disconnect()
        {
            receiveSubr_.unsubscribe();
        }

        // ############# Below are the input functions ################

        HAL::IO_TYPE_t TopicIO::getInputType() const
        {
            return HAL::IO_TYPE_t::TOPIC;
        }

        bool TopicIO::setInputParam(HAL::IO_PARAM_t param, int32_t value)
        {
            LOG_MSG("TopicIO setInputParam: Something attempted to change param values. This is not supported in topicIO! Param: %d, Value: %d\n", param, value);
            return false;
        }

        size_t TopicIO::readable()
        {
            // VRBS_MSG("Buffer front: %d, back: %d, size: %d, this: %d\n", receiveBuffer_.getFront(), receiveBuffer_.getBack(), receiveBuffer_.size(), this);
            return receiveBuffer_.size();
        }

        size_t TopicIO::readData(void *data, size_t size, bool endTransfer)
        {

            VRBS_MSG("Reading %d bytes from buffer. Buffer size: %d. This: %d\n", size, receiveBuffer_.size(), this);

            size_t i = 0;
            for (; i < size && receiveBuffer_.size() > 0; i++)
            {
                receiveBuffer_.takeFront(static_cast<uint8_t *>(data)[i]);
            }
            return i;
        }

        // ############# Below are the output functions ################

        HAL::IO_TYPE_t TopicIO::getOutputType() const
        {
            return HAL::IO_TYPE_t::TOPIC;
        }

        bool TopicIO::setOutputParam(HAL::IO_PARAM_t param, int32_t value)
        {
            LOG_MSG("TopicIO setOutputParam: Something attempted to change param values. This is not supported in topicIO! Param: %d, Value: %d\n", param, value);
            return false;
        }

        size_t TopicIO::writable()
        {
            return 255;
        }

        size_t TopicIO::writeData(const void *data, size_t size, bool endTransfer)
        {

            if (size > 255) // Too big to send
                return 0;

            VRBS_MSG("Sending %d bytes through topic. This: %d\n", size, this);

            receiveSubr_.publish(Core::ListExtern<uint8_t>((uint8_t *)data, size));

            return size;
        }

    }

}
