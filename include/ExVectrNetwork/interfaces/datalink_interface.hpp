#ifndef EXVECTRNETWORK_INTERFACES_DATALINKINTERFACE_H_
#define EXVECTRNETWORK_INTERFACES_DATALINKINTERFACE_H_

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
         * @brief Interface for the datalink layer. Inhereting classes must implement the dataframeReceiveFunc function.
         */
        class Datalink_Interface
        {
        protected:

            ///@brief Topic where dataframes received by physical, are published as an object offering a list interface. This object must be copied by the subscribers.
            Core::Topic<Core::List<uint8_t>> receiveTopic_;
            ///@brief This subscriber receives dataframes that are to be sent over the physical layer.
            Core::Callback_Subscriber<Core::List<uint8_t>, Datalink_Interface> transmitSubr_;

        public:

            Datalink_Interface();

            /**
             * @brief The topic where the received data frames are published to. (Datalink -> Network)
             * @return the receive topic
             */
            Core::Topic<Core::List<uint8_t>> &getReceiveTopic();

            /** 
             * @brief Sets the topic where data frames are published to, to be transmitted over the physical layer. (Network -> Datalink)
             * @note This object subscribes to the given topic.
             * @param transmitTopic The topic where dataframes to be transmitted are published to.
             */
            void setTransmitTopic(Core::Topic<Core::List<uint8_t>> &transmitTopic);

            /**
             * @brief Unsubscribes from all topics sending frames.
             */
            void removeTransmitTopic();

        protected:
            /**
             * @brief This function receives published dataframes from the transmit topic.
             * @param item Interface to dataframe containing list
             */
            virtual void dataframeReceiveFunc(const Core::List<uint8_t> &item) = 0;

        };

    }

}

#endif