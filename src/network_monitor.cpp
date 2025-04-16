#include "ExVectrCore/list.hpp"
#include "ExVectrCore/list_static.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/interfaces/network_interface.hpp"

#include "ExVectrNetwork/network_monitor.hpp"


namespace VCTR
{

    namespace Net
    {

        
        NetworkMonitor::NetworkMonitor(Net::Network_Interface &networkInterface, int64_t sendInterval, int64_t timeoutInterval) :
            Core::Task_Periodic("Network Monitor", sendInterval)
        {
            transmitTopicPub_.subscribe(networkInterface.getTransmitTopic());
            receiveSubr_.subscribe(networkInterface.getReceiveTopic());
            Core::getSystemScheduler().addTask(*this);
        }

        void NetworkMonitor::setSendInterval(int64_t interval) {
            sendInterval_ = interval;
        }
        void NetworkMonitor::setTimeoutInterval(int64_t interval) {
            timeoutInterval_ = interval;
        }

        bool NetworkMonitor::isNodeReachable(uint16_t nodeAddress) {
            for (int i = 0; i < nodeList_.size(); i++) {
                if (nodeList_[i].nodeAddress == nodeAddress) {
                    return true;
                }
            }
            return false;
        }

        void NetworkMonitor::checkPacket(const NetworkPacket& packet) {

            if (packet.type == NetworkPacketType::HEARTBEAT) {
                for (int i = 0; i < nodeList_.size(); i++) {
                    if (nodeList_[i].nodeAddress == packet.srcAddress) {
                        nodeList_[i].lastSeen = Core::NOW();
                        return;
                    }
                }
                NodeInfo nodeInfo;
                nodeInfo.nodeAddress = packet.srcAddress;
                nodeInfo.lastSeen = Core::NOW();
                nodeList_.appendIfNotInListArray(nodeInfo);
            }

        }

        void NetworkMonitor::taskThread() {

            // Send out our heartbeat packet
            NetworkPacket packet;
            packet.type = NetworkPacketType::HEARTBEAT;
            packet.hops = 1;
            packet.dstAddress = 0xFFFF; // Broadcast
            packet.payload.placeBack(5); //Dummy payload
            transmitTopicPub_.publish(packet); // Send the packet

            // Handle received packets
            for (int i = 0; i < receiveSubr_.size(); i++) {
                checkPacket(receiveSubr_[i]); // Check the packet
            }
            receiveSubr_.clear();

            // Check for timeouts
            auto time = Core::NOW();
            for (int i = 0; i < nodeList_.size(); i++) {
                if (time - nodeList_[i].lastSeen > timeoutInterval_) {
                    nodeList_.removeAtIndex(i); // Remove the node from the list if it has timed out
                    i--;
                }
            }

        }


    }

}