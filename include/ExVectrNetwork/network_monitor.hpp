#ifndef EXVECTRNETWORK_NETWORKMONITOR_HPP_
#define EXVECTRNETWORK_NETWORKMONITOR_HPP_

#include "ExVectrCore/list.hpp"
#include "ExVectrCore/list_static.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/interfaces/network_interface.hpp"


namespace VCTR
{

    namespace Net
    {

        /**
         * @brief This class is used to monitor connections between nodes in the network. Each node should have one of these objects.
         * @note Using this adds a small amount of constant traffic to the network.
         * @details This class broadcasts a packet and listens for others, when it receives a packet it uses the sender to determine if the node is reachable.
         */
        class NetworkMonitor : public Core::Task_Periodic
        {
        private:
            
            Core::Topic_Publisher<NetworkPacket> transmitTopicPub_;
            Core::Buffer_Subscriber<NetworkPacket, 10> receiveSubr_;

            int64_t timeoutInterval_ = 2 * Core::SECONDS;
            int64_t sendInterval_ = 0.5 * Core::SECONDS;

            int64_t lastSend_ = 0;

            struct NodeInfo
            {
                uint16_t nodeAddress;
                int64_t lastSeen;

                // Checks if the nodes are the same. Ignores the lastSeen time.
                bool operator==(const NodeInfo& other) const
                {
                    return nodeAddress == other.nodeAddress;
                }

            };
            Core::ListArray<NodeInfo> nodeList_;


        public:

            /**
             * * @brief Construct a new Network Monitor object.
             * * @param networkInterface The network interface to use for sending and receiving packets.
             * * @param sendInterval The interval at which to send packets. Default is 0.5 seconds.
             * * @param timeoutInterval The interval at which to consider a node unreachable. Default is 2 seconds. Should be a multiple of sendInterval.
             */
            NetworkMonitor(Net::Network_Interface &networkInterface, int64_t sendInterval = 0.5 * Core::SECONDS, int64_t timeoutInterval = 2 * Core::SECONDS);

            void setSendInterval(int64_t interval);
            void setTimeoutInterval(int64_t interval);

            /**
             * @brief Checks if the given node is reachable.
             * @note Due to the timeout, it can take some time until a node is considered unreachable.
             * @param nodeAddress The address of the node to check.
             * @return true if the node is reachable, false otherwise.
             */
            bool isNodeReachable(uint16_t nodeAddress);

        private:

            //void taskInit() override;

            void taskThread() override;

            void checkPacket(const NetworkPacket& packet);

        };

    }

}

#endif