#ifndef EXVECTRNETWORK_TOPICIO_H_
#define EXVECTRNETWORK_TOPICIO_H_

#include "ExVectrCore/list_buffer.hpp"

#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrHAL/digital_io.hpp"

namespace VCTR
{

    namespace Net
    {

        /**
         * @brief A class implementing an IO interface using topics
         */
        class TopicIO: public HAL::DigitalIO {
        private:
            
            /// Where the bytes are received
            Core::Callback_Subscriber<const uint8_t*, TopicIO> receiveSubr_;
            
            /// List used to buffer received data.
            Core::ListBuffer<uint8_t, 1024> receiveBuffer_;

            uint8_t numReceiving_ = 0;

        public:

            TopicIO();

            /**
             * @brief Ctor what also subscribes to the given topic.
             * @param topic Topic used to transfer data.
             */
            TopicIO(Core::Topic<const uint8_t*>& topic);

            /**
             * @brief Connects the given topicIO to this one. Can be connected to multiple.
             * @param topicIO 
             */
            void connectTopicIO(Core::Topic<const uint8_t*>& topic);

            /**
             * @brief Disconnects given topicIO.
             * @param topicIO 
             */
            void disconnect(Core::Topic<const uint8_t*>& topic);

            /**
             * @brief Disconnects all connected topicIO
             */
            void disconnect();


            // ############# Below are the input functions ################

            /**
             * @brief Gets the type of bus this is.
             * @return IO_TYPE_t enum.
             */
            HAL::IO_TYPE_t getInputType() const override;

            /**
             * @brief Changes given parameter. 
             * @param param What parameter to change.
             * @param value What to change parameter to.
             * @return True if successfull and parameter is supported.
             */
            bool setInputParam(HAL::IO_PARAM_t param, int32_t value) override;

            /**
             * @returns the number of bytes available to read. Or 1 or 0 for boolean.
            */
            size_t readable() override;

            /**
             * @brief Reads the data and places it into the given data pointer.
             * @param data Pointer to where to place read data.
             * @param bytes Number of bytes to read. Will be limited to this number.
             * @param endTransfer Set to false if doing mulitple reads. Last read should have endTransfer set to true.
             * 
             * @return number of bytes actually read and placed into data pointer.
            */
            size_t readData(void* data, size_t size, bool endTransfer = true) override;


            // ############# Below are the output functions ################

            /**
             * @brief Gets the type of bus this is.
             * @return IO_TYPE_t enum.
             */
            HAL::IO_TYPE_t getOutputType() const override;

            /**
             * @brief Changes given parameter. 
             * @param param What parameter to change.
             * @param value What to change parameter to.
             * @return True if successfull and parameter is supported.
             */
            bool setOutputParam(HAL::IO_PARAM_t param, int32_t value) override;

            /**
             * @returns number of bytes that can be written. -1 means no limit to data size.
            */
            int32_t writable() override;

            /**
             * @brief Writes the data from data pointer.
             * @param data Pointer to data.
             * @param bytes Number of bytes to output.
             * @param endTransfer Set to false if doing mulitple writes. Last write should have endTransfer set to true.
             * 
             * @return number of bytes actually written.
            */
            size_t writeData(const void* data, size_t size, bool endTransfer = true) override;

        private:   

            void receiveItem(const uint8_t * const& item);

        };

    }

}

#endif