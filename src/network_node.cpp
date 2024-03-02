#ifndef EXVECTRNETWORK_NETWORK_H_
#define EXVECTRNETWORK_NETWORK_H_

#include "ExVectrCore/list.hpp"
#include "ExVectrCore/list_static.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/print.hpp"

#include "ExVectrNetwork/datalink.hpp"
#include "ExVectrNetwork/network_node.hpp"

#define EXVECTR_DEBUG_VRBS_ENABLE

namespace VCTR
{

    namespace Net
    {

        NetworkNode::NetworkNode(uint16_t nodeAddress)
        {

            nodeAddress_ = nodeAddress;

            transmitTopicSubr_.setCallback(this, &NetworkNode::sendPacket);
            transmitTopicSubr_.subscribe(transmitTopic_);

            linkReceiveSubr_.setCallback(this, &NetworkNode::receivePacket);
        }

        NetworkNode::~NetworkNode()
        {
        }

        void NetworkNode::setNodeAddress(uint16_t nodeAddress)
        {
            nodeAddress_ = nodeAddress;
        }

        uint16_t NetworkNode::getNodeAddress() const
        {
            return nodeAddress_;
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
            packetBytes.setSize(packet.payloadLength + 8);
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

        Core::Topic<NetworkPacket> &NetworkNode::getTransmitTopic()
        {
            return transmitTopic_;
        }

        Core::Topic<NetworkPacket> &NetworkNode::getReceiveTopic()
        {
            return receiveTopic_;
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
                return;
            }

            if (packet.dstAddress == nodeAddress_) // If this packet is for this node, publish it directly to the receive topic.
            {
                receiveTopic_.publish(packet);
            }
            else if (packet.dstAddress == 0xFFFF) // Broadcast packet or we are not addressed. Publish to receive topic and route.
            {
                receiveTopic_.publish(packet);
                routePacket(packet);
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
            packet.payloadLength = data[7];

            if (data.size() != packet.payloadLength + 8)
            {
                LOG_MSG("Data buffer wrong size! \n");
                return false;
            }

            for (size_t i = 0; i < packet.payloadLength; i++)
            {
                packet.payload[i] = data[8 + i];
            }

            // Calculate checksum
            uint8_t checksum = 0;
            checksum += uint8_t(packet.type);
            checksum += packet.hops;
            checksum += packet.dstAddress >> 8;
            checksum += packet.dstAddress & 0xFF;
            checksum += packet.srcAddress >> 8;
            checksum += packet.srcAddress & 0xFF;
            checksum += packet.payloadLength;
            checksum += networkVersion;
            for (size_t i = 0; i < packet.payloadLength; i++)
            {
                checksum += packet.payload[i];
            }

            if (checksum != data[6])
            {
                LOG_MSG("Checksum failed! Expected: %d, Is: %d \n", checksum, data[6]);
                return false;
            }

            return true;
        }

        bool NetworkNode::packPacket(const NetworkPacket &packet, Core::List<uint8_t> &data)
        {

            VRBS_MSG("Packing packet. Payload len: %d.\n", packet.payloadLength);
            if (data.size() < packet.payloadLength + 8)
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
            data[7] = packet.payloadLength;

            for (size_t i = 0; i < packet.payloadLength; i++)
            {
                data[8 + i] = packet.payload[i];
            }

            // Calculate checksum
            uint8_t checksum = 0;
            checksum += uint8_t(packet.type);
            checksum += packet.hops;
            checksum += packet.dstAddress >> 8;
            checksum += packet.dstAddress & 0xFF;
            checksum += packet.srcAddress >> 8;
            checksum += packet.srcAddress & 0xFF;
            checksum += packet.payloadLength;
            checksum += networkVersion;

            for (size_t i = 0; i < packet.payloadLength; i++)
            {
                checksum += packet.payload[i];
            }

            data[6] = checksum;

            return true;
        }

        void NetworkNode::routePacket(const NetworkPacket &packet)
        {
        }

    }

}

#endif