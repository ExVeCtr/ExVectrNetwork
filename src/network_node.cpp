#include "ExVectrCore/cyclic_checksum.hpp"
#include "ExVectrCore/list.hpp"
#include "ExVectrCore/list_static.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/print.hpp"

#include "ExVectrNetwork/datalink.hpp"
#include "ExVectrNetwork/structs/network_packet.hpp"
#include "ExVectrNetwork/network_node.hpp"

namespace VCTR
{

    namespace Net
    {

        NetworkNode::NetworkNode(uint16_t nodeAddress) : Network_Interface(nodeAddress)
        {

            transmitTopicSubr_.setCallback(this, &NetworkNode::sendPacket);
            transmitTopicSubr_.subscribe(transmitTopic_);

            linkReceiveSubr_.setCallback(this, &NetworkNode::receivePacket);
        }

        NetworkNode::NetworkNode(uint16_t nodeAddress, Datalink_Interface &datalink) : NetworkNode(nodeAddress)
        {
            setDatalink(datalink);
        }

        NetworkNode::~NetworkNode()
        {
        }

        void NetworkNode::setDatalink(Datalink_Interface &datalink)
        {
            linkTransmitTopic_.unsubscribeAll(); // Remove previous datalink.

            linkReceiveSubr_.subscribe(datalink.getReceiveTopic()); // Subscribe to new datalink.
            datalink.setTransmitTopic(linkTransmitTopic_);
        }

        void NetworkNode::sendPacket(const NetworkPacket &packet)
        {   

            VRBS_MSG("Sending Packet! Pointer: %d\n", this);

            auto packetSend = packet;
            packetSend.srcAddress = nodeAddress_;
            packetSend.type = NetworkPacketType::DATA;

            if (nodeAddress_ == packetSend.dstAddress)
            { // If this packet is for this node, publish it directly to the receive topic.
                receiveTopic_.publish(packet);
                return;
            }

            Core::ListArray<uint8_t> packetBytes;
            packetBytes.setSize(packet.payload.size() + 8);
            if (!packPacket(packetSend, packetBytes))
            {
                LOG_MSG("Failed to pack packet! \n");
                return;
            }

            //Print the contents of the packet array
            /*VRBS_MSG("Send Packet bytes: \n");
            for (size_t i = 0; i < packetBytes.size(); i++)
            {
                VRBS_MSG("%d, %d \n", i, packetBytes[i]);
            }*/
            //LOG_MSG("\n");
            linkTransmitTopic_.publish(packetBytes);
        }

        void NetworkNode::receivePacket(const Core::List<uint8_t> &data)
        {

            VRBS_MSG("Received a packet. Size: %d Pointer: %d\n", data.size(), this);

            //Print the contents of the packet array
            /*VRBS_MSG("Receive Packet bytes: \n");
            for (size_t i = 0; i < data.size(); i++)
            {
                VRBS_MSG("%d, %d \n", i, data[i]);
            }*/

            NetworkPacket packet;
            if (!unpackPacket(packet, data))
            {
                VRBS_MSG("Failed to unpack packet! \n");
                return;
            }

            if (packet.dstAddress == nodeAddress_ || packet.dstAddress == UINT16_MAX) // If this packet is for this node, publish it directly to the receive topic.
            {
                //Decrement hops if not zero
                if (packet.hops > 0)
                    packet.hops--;
                
                //Send packet to receive topic (Network -> Transport)
                receiveTopic_.publish(packet);
            }
        }

        bool NetworkNode::unpackPacket(NetworkPacket &packet, const Core::List<uint8_t> &data)
        {

            VRBS_MSG("Unpacking packet. Length: %d.\n", data.size());

            packet.type = NetworkPacketType(data[0]);
            packet.hops = data[1];
            packet.dstAddress = (data[2] << 8) | data[3];
            packet.srcAddress = (data[4] << 8) | data[5];
            // Byte 6 is checksum
            uint8_t payloadSize = data[7];

            VRBS_MSG("Packet type: %d, Hops: %d, Dst: %d, Src: %d, Payload size: %d.\n", packet.type, packet.hops, packet.dstAddress, packet.srcAddress, payloadSize);

            if (data.size() != size_t(payloadSize + 8))
            {
                LOG_MSG("Data buffer wrong size! Packet size: %d, Data size: %d \n", packet.payload.size() + 8, data.size());
                return false;
            }

            packet.payload.clear();
            for (size_t i = 0; i < payloadSize; i++)
            {
                packet.payload.placeBack(data[8 + i]);
            }

            // Calculate checksum
            uint8_t checksum = 0;//Core::computeCrc(data, 0);
            for (size_t i = 0; i < data.size(); i++)
            {
                if (i == 6) // Skip checksum byte
                    continue;
                checksum += data[i];
            }

            if (checksum != data[6])
            {
                LOG_MSG("Checksum failed! Expected: %d, Is: %d \n", data[6], checksum);
                return false;
            }

            return true;
        }

        bool NetworkNode::packPacket(const NetworkPacket &packet, Core::List<uint8_t> &data)
        {

            VRBS_MSG("Packing packet. Payload len: %d.\n", packet.payload.size());
            if (data.size() < packet.payload.size() + 8)
            {
                LOG_MSG("Data buffer too small! \n");
                return false;
            }

            data[0] = uint8_t(packet.type);
            data[1] = packet.hops;
            data[2] = packet.dstAddress >> 8;
            data[3] = packet.dstAddress & 0xFF;
            data[4] = packet.srcAddress >> 8;
            data[5] = packet.srcAddress & 0xFF;
            // Byte 6 is checksum
            data[6] = 0;
            data[7] = packet.payload.size();

            for (size_t i = 0; i < packet.payload.size(); i++)
            {
                data[8 + i] = packet.payload[i];
            }

            // Calculate checksum
            uint8_t checksum = 0;//Core::computeCrc(data, 0);
            for (size_t i = 0; i < data.size(); i++)
            {
                checksum += data[i];
            }

            data[6] = checksum;

            return true;
        }

    }

}