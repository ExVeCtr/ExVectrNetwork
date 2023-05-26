#include "ExVectrCore/list_buffer.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/print.hpp"

#include "ExVectrHAL/io.hpp"

#include "ExVectrNetwork/physicallayers/topic_io.hpp"

namespace VCTR
{

    namespace Net
    {   

        TopicIO::TopicIO() 
        {
            receiveSubr_.setCallbackObject(this);
            receiveSubr_.setCallbackFunction(&TopicIO::receiveItem);
        }

        TopicIO::TopicIO(Core::Topic<const uint8_t*>& topic) : TopicIO()
        {
            receiveSubr_.subscribe(topic);
        }

        void TopicIO::receiveItem(const uint8_t *const &item)
        {
            if (numReceiving_ == 0)
            {
                numReceiving_ = item[0];
            }
            else
            {
                for (size_t i = 0; i < numReceiving_; i++)
                    receiveBuffer_.placeBack(item[i], true);
                numReceiving_ = 0;
            }
        }

        void TopicIO::connectTopicIO(Core::Topic<const uint8_t*>& topic)
        {
            receiveSubr_.subscribe(topic);
        }

        void TopicIO::disconnect(Core::Topic<const uint8_t*>& topic)
        {
            receiveSubr_.unsubcribe(topic);
        }

        void TopicIO::disconnect()
        {
            receiveSubr_.unsubcribe();
        }

        // ############# Below are the input functions ################

        HAL::IO_TYPE_t TopicIO::getInputType() const
        {
            return HAL::IO_TYPE_t::TOPIC;
        }

        bool TopicIO::setInputParam(HAL::IO_PARAM_t param, int32_t value)
        {
            Core::printW("TopicIO setInputParam: Something attempted to change param values. This is not supported in topicIO! Param: %d, Value: %d\n", param, value);
            return false;
        }

        size_t TopicIO::readable()
        {
            return receiveBuffer_.size();
        }

        size_t TopicIO::readData(void *data, size_t size, bool endTransfer)
        {
            size_t i = 0;
            for (; i < size && receiveBuffer_.size() > 0; i++)
            {
                receiveBuffer_.takeFront(((uint8_t *)data)[i]);
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
            Core::printW("TopicIO setOutputParam: Something attempted to change param values. This is not supported in topicIO! Param: %d, Value: %d\n", param, value);
            return false;
        }

        int32_t TopicIO::writable()
        {
            return 255;
        }

        size_t TopicIO::writeData(const void *data, size_t size, bool endTransfer)
        {

            size_t bytesSent = 0;
            while (size > 0)
            {

                uint8_t len = size;
                if (len > 255)
                    len = 255;

                receiveSubr_.publish(&len);
                receiveSubr_.publish((uint8_t *)(size_t(data) + bytesSent));

                bytesSent += len;
                size -= len;
            }

            return bytesSent;
        }

    }

}
