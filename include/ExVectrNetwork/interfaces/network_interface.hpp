#ifndef EXVECTRNETWORK_INTERFACES_NETWORKINTERFACE_HPP_
#define EXVECTRNETWORK_INTERFACES_NETWORKINTERFACE_HPP_

#include "ExVectrNetwork/interfaces/datalink_interface.hpp"
#include "ExVectrNetwork/structs/network_packet.hpp"

namespace VCTR
{

    namespace Net
    {

        /**
         * @brief Network layer class. This class takes care of routing packets to their destination.
         */
        class Network_Interface
        {
        protected:

            /// The address of this node.
            uint16_t nodeAddress_;

            /// Where received packets for this node are published.
            Core::Topic<NetworkPacket> receiveTopic_;

            /// Where packets to be sent are published.
            Core::Topic<NetworkPacket> transmitTopic_;

        public:

            /**
             * @brief Construct a new Network Node object.
             *
             * @param nodeAddress The address of this node. Set to 0 to only receive packets.
             */
            Network_Interface(uint16_t nodeAddress) : nodeAddress_(nodeAddress) {}

            /**
             * @brief Set the address of this node.
             *
             * @param nodeAddress The address of this node.
             */
            void setNodeAddress(uint16_t nodeAddress) {
                nodeAddress_ = nodeAddress;
            }

            /**
             * @brief Get the address of this node.
             *
             * @return uint16_t The address of this node.
             */
            uint16_t getNodeAddress() const {
                return nodeAddress_;
            }

            /**
             * @brief Get the transmit topic to publish packets to be sent.
             *
             */
            Core::Topic<NetworkPacket> &getTransmitTopic() {
                return transmitTopic_;
            }

            /**
             * @brief Get the receive topic to subscribe to receive packets.
             *
             */
            Core::Topic<NetworkPacket> &getReceiveTopic() {
                return receiveTopic_;
            }

        };

    }

}

#endif