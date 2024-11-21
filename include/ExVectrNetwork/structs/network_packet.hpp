#ifndef EXVECTRNETWORK_STRUCTS_NETWORKPACKET_HPP_
#define EXVECTRNETWORK_STRUCTS_NETWORKPACKET_HPP_

#include "ExVectrCore/list.hpp"
#include "ExVectrCore/list_static.hpp"

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
            NetworkPacketType type = NetworkPacketType::DATA;
            /// Number of hops this packet can still take. Will be decremented by 1 each time it is forwarded. Set to 1 to prevent forwarding.
            uint8_t hops = 1;
            /// Destination address. Who receives this? Set to 0xFFFF to broadcast.
            uint16_t dstAddress;
            /// Source address. Who sent this?
            uint16_t srcAddress;
            /// Checksum. Used to verify the integrity of the packet. Calculated as the sum of all bytes in the packet only excluding the checksum byte plus network version number.
            uint8_t checksum;
            /// The data carried by this packet.
            Core::ListBuffer<uint8_t, 200> payload;
        };

    }

}

#endif