#ifndef EXVECTRNETWORK_NETWORKNODE_HPP_
#define EXVECTRNETWORK_NETWORKNODE_HPP_

#include "ExVectrCore/list.hpp"
#include "ExVectrCore/list_static.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrNetwork/interfaces/datalink_interface.hpp"
#include "ExVectrNetwork/interfaces/network_interface.hpp"
#include "ExVectrNetwork/structs/network_packet.hpp"

namespace VCTR
{

    namespace Net
    {

        /**
         * @brief Network layer class. This class takes care of routing packets to their destination.
         */
        class NetworkNode : public Network_Interface
        {
        private:
            /// Network version number. Used to calculate checksum and prevent incompatible networks from communicating.
            static constexpr uint8_t networkVersion = 1;


            /// Where packets to be sent are published.
            Core::Callback_Subscriber<NetworkPacket, NetworkNode> transmitTopicSubr_;

            /// Where received packets from datalink layer are published.
            Core::Callback_Subscriber<Core::List<uint8_t>, NetworkNode> linkReceiveSubr_;
            /// Where packets to be sent to datalink layer are published.
            Core::Topic<Core::List<uint8_t>> linkTransmitTopic_;

            
            /// The list of nodes that this node can reach.
            // Core::ListStatic<uint16_t, 10> neighbours_;

        public:
            /**
             * @brief Construct a new Network Node object.
             *
             * @param nodeAddress The address of this node. Set to 0 to only receive packets.
             */
            NetworkNode(uint16_t nodeAddress);

            /**
             * @brief Construct a new Network Node object.
             *
             * @param nodeAddress The address of this node. Set to 0 to only receive packets.
             * @param datalink The datalink layer to use for sending and receiving packets.
             */
            NetworkNode(uint16_t nodeAddress, Datalink_Interface &datalink);

            ~NetworkNode();

            /**
             * @brief Set the datalink layer to use for sending and receiving packets.
             *
             * @param datalink The datalink layer to use.
             */
            void setDatalink(Datalink_Interface &datalink);

            /**
             * @brief Send a packet to the given destination address.
             *
             * @param packet The packet to send.
             * @param dstAddress The destination address.
             */
            void sendPacket(const NetworkPacket &packet);

        private:

            /**
             * @brief Unpacks a packet from a list of bytes.
             *
             * @param packet The packet to unpack.
             * @param data The data to unpack from.
             * @return true if successful, false if not.
             */
            bool unpackPacket(NetworkPacket &packet, const Core::List<uint8_t> &data);

            /**
             * @brief Packs a packet into a list of bytes.
             *
             * @param packet The packet to pack.
             * @param data The data to pack into.
             * @return true if successful, false if not.
             */
            bool packPacket(const NetworkPacket &packet, Core::List<uint8_t> &data);
            
            /**
             * @brief Receives a packet from the datalink layer.
             *
             * @param data The data received.
             */
            void receivePacket(const Core::List<uint8_t> &data);

            /**
             * @brief Routes a packet to its destination.
             *
             * @param packet The packet to route.
             */
            void routePacket(const NetworkPacket &packet);

        };

    }

}

#endif