#ifndef EXVECTRNETWORK_NETWORKNODE_HPP_
#define EXVECTRNETWORK_NETWORKNODE_HPP_

#include "ExVectrCore/list.hpp"
#include "ExVectrCore/list_static.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrNetwork/interfaces/datalink_interface.hpp"

namespace VCTR
{

    namespace Net
    {

        enum class NetworkPacketType : uint8_t
        {
            DATA,     // Packet data is for the application layer.
            ACK,      // Packet data is an acknowledgement for a packet.
            NACK,     // Packet data is a negative acknowledgement for a packet.
            HEARTBEAT // Packet data is a heartbeat. Broadcasted after an interval since last sent packet.
        };

        /**
         * @brief   Network packet structure.
         * @note    Raw data structure follows this format: [type, hops, dstAddress, srcAddress, checksum, dataLength, data...]
         *          Usually only packet hops, dstAddress, payloadLength and payload are used by the application layer. The rest is handled by the network layer.
         */
        struct NetworkPacket
        {
            /// Packet type. What is this packet for?
            NetworkPacketType type;
            /// Number of hops this packet can still take. Will be decremented by 1 each time it is forwarded. Set to 1 to prevent forwarding.
            uint8_t hops = 1;
            /// Destination address. Who receives this? Set to 0xFFFF to broadcast.
            uint16_t dstAddress;
            /// Source address. Who sent this?
            uint16_t srcAddress;
            /// Checksum. Used to verify the integrity of the packet. Calculated as the sum of all bytes in the packet only excluding the checksum byte plus network version number.
            uint8_t checksum;
            /// The length of the data carried by this packet in bytes.
            uint8_t payloadLength;
            /// The data carried by this packet.
            Core::ListStatic<uint8_t, 200> payload;
        };

        /**
         * @brief Network layer class. This class takes care of routing packets to their destination.
         */
        class NetworkNode
        {
        private:
            /// Network version number. Used to calculate checksum and prevent incompatible networks from communicating.
            static constexpr uint8_t networkVersion = 0x01;

            /// Where received packets for this node are published.
            Core::Topic<NetworkPacket> receiveTopic_;

            /// Where packets to be sent are published.
            Core::Topic<NetworkPacket> transmitTopic_;
            Core::Callback_Subscriber<NetworkPacket, NetworkNode> transmitTopicSubr_;

            /// Where received packets from datalink layer are published.
            Core::Callback_Subscriber<Core::List<uint8_t>, NetworkNode> linkReceiveSubr_;
            /// Where packets to be sent to datalink layer are published.
            Core::Topic<Core::List<uint8_t>> linkTransmitTopic_;

            /// The address of this node.
            uint16_t nodeAddress_;

            /// The list of nodes that this node can reach.
            // Core::ListStatic<uint16_t, 10> neighbours_;

        public:
            /**
             * @brief Construct a new Network Node object.
             *
             * @param nodeAddress The address of this node. Set to 0 to only receive packets.
             */
            NetworkNode(uint16_t nodeAddress);

            ~NetworkNode();

            /**
             * @brief Set the address of this node.
             *
             * @param nodeAddress The address of this node.
             */
            void setNodeAddress(uint16_t nodeAddress);

            /**
             * @brief Get the address of this node.
             *
             * @return uint16_t The address of this node.
             */
            uint16_t getNodeAddress() const;

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

            /**
             * @brief Get the transmit topic to publish packets to be sent.
             *
             */
            Core::Topic<NetworkPacket> &getTransmitTopic();

            /**
             * @brief Get the receive topic to subscribe to receive packets.
             *
             */
            Core::Topic<NetworkPacket> &getReceiveTopic();

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