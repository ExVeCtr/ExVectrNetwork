#ifndef EXVECTRNETWORK_DATALINK_H_
#define EXVECTRNETWORK_DATALINK_H_

#include "ExVectrCore/list.hpp"
#include "ExVectrCore/list_buffer.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrHAL/digital_io.hpp"

namespace VCTR
{

    namespace Net
    {

        /**
         * @brief   Datalink layer class. This class controls media access to a physical layer and allows any size data to be sent over a physical layer using the topic subscriber interface.
         * @note    The physical layer must support ending a transfer.
         */
        class Datalink : public Core::Task_Periodic
        {
        public:

            ///@brief Maximum length a data frame can be.
            static constexpr size_t dataLinkMaxFrameLength = 200; 

        private:

            enum class PhysicalHeader {
                BLOCK = 0,
                FREE,
                DATA
            };

            ///@brief If the physicallayer is currently blocked. Cannot send during this time.
            bool physicalBlocked_ = false;
            ///@brief Timestamp of the last time the physical layer was blocked. Use to release after a set amount of time.
            int64_t physicalBlockTimestamp_ = 0;
            ///@brief How long to wait until auto releasing the physical layer after the last received data.
            int64_t physicalReleaseTime_ = 100*Core::MILLISECONDS;

            ///@brief if we are currently transmitting data.
            bool transmitting_ = false;
            ///@brief The number of bytes to transmit.
            size_t numBytesTransmit_ = 0;

            ///@brief If we are currently receiving data.
            bool receiving_ = false;
            ///@brief number of bytes to still be received.
            size_t numBytesReceive_ = 0;

            ///@brief The physical layer that offers IO interface for reading/writing.
            HAL::DigitalIO *physicalLayer_ = nullptr;

            ///@brief Topic where dataframes received by physical, are published as an object offering a list interface. This object must be copied by the subscribers.
            Core::Topic<Core::List<uint8_t>> receiveTopic_;
            ///@brief This subscriber receives dataframes that are to be sent over the physical layer.
            Core::Callback_Subscriber<Core::List<uint8_t>, Datalink> transmitSubr_;

            ///@brief Buffer for data to transmit.
            Core::ListBuffer<uint8_t, dataLinkMaxFrameLength * 5> transmitBuffer_;
            ///@brief Buffer for data to transmit.
            Core::ListBuffer<uint8_t, dataLinkMaxFrameLength * 5> receiveBuffer_;
            

        public:

            /**
             * @brief Constructor. This requires an IO interface of an object, usually a physical layer but can be a topic offering this interface.
             * @note The physical layer is simply a medium to transfer raw data, this can be a LoRa device like SX1280 or UART, SPI etc.
             * @param physicalLayerDevice The object used as the physical layer.
             */
            Datalink(HAL::DigitalIO &physicalLayerDevice, Core::Scheduler &scheduler = Core::getSystemScheduler());

            /**
             * @brief The topic where the received data frames are published to.
             * @return the receive topic
             */
            Core::Topic<Core::List<uint8_t>> &getReceiveTopic();

            /**
             * @brief Sets the topic where data frames are published to, to be transmitted over the physical layer.
             * @note This object subscribes to the given topic.
             * @param transmitTopic The topic where dataframes to be transmitted are published to.
             */
            void setTransmitTopic(Core::Topic<Core::List<uint8_t>> &transmitTopic);

            /**
             * @brief Unsubscribes from all topics sending frames.
             */
            void removeTransmitTopic();

            /**
             * @brief Removes the given transmit topic from subscription.
             * @param transmitTopic What topic to stop transmitting frames from.
             * @return true if unsubscribed, false if already subscribed.
             */
            void removeTransmitTopic(Core::Topic<Core::List<uint8_t>> &transmitTopic);

            /**
             * @brief Set the amount of time to wait for a blocked channel to become free.
             * @note    Set this to a multiple of the expected time it takes for data to be transfered over the channel, but as low as possible.
             *          Example: LoRa can take multiple seconds -> timeout of 5 seconds.
             *          Warning, something going wrong on the channel can cause datatransfer to be blocked for the amount of time.
             * @param time 
             */
            void setPhysicalReleaseTimeout(int64_t time);

        private:
            /**
             * @brief This function receives published dataframes from the transmit topic.
             * @param item Interface to dataframe containing list
             */
            void dataframeReceiveFunc(const Core::List<uint8_t> &item);

            /**
             * @brief Checks the IO if it has data to read.
             */
            void taskCheck() override;

            /**
             * @brief Initialised datalink
             */
            void taskInit() override;

            /**
             * @brief Reads and writes data to the physical layer.
             */
            void taskThread() override;

        };

    }

}

#endif