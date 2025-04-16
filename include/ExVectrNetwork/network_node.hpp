#ifndef EXVECTRNETWORK_NETWORKNODE_HPP_
#define EXVECTRNETWORK_NETWORKNODE_HPP_

#include "ExVectrCore/list.hpp"
#include "ExVectrCore/list_static.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/task_types.hpp"

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
        class NetworkNode : public Network_Interface, public Core::Task_Periodic
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

            /// If we havent heard from a node in this time, we consider it unreachable. Should be 4 times the sendInterval.
            int64_t timeoutInterval_ = 1 * Core::SECONDS;
            /// If the time since we last sent a packet is greater than this, we send a heartbeat packet to show we are still connected.
            int64_t sendInterval_ = 0.25 * Core::SECONDS;

            /// The last time we sent a packet. Used to determine if we need to send a heartbeat packet.
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
            /// @brief The list of nodes that this node can reach.
            Core::ListArray<NodeInfo> nodeList_;


        public:
            /**
             * @brief Construct a new Network Node object.
             *
             * @param nodeAddress The address of this node. Set to 0 to only receive packets.
             */
            NetworkNode(uint16_t nodeAddress, int64_t disconnectTimeout = 1 * Core::SECONDS);

            /**
             * @brief Construct a new Network Node object.
             *
             * @param nodeAddress The address of this node. Set to 0 to only receive packets.
             * @param datalink The datalink layer to use for sending and receiving packets.
             */
            NetworkNode(uint16_t nodeAddress, Datalink_Interface &datalink, int64_t disconnectTimeout = 1 * Core::SECONDS);

            //~NetworkNode();

            /**
             * @brief Checks if the given node is reachable.
             * @note Due to the timeout, it can take some time until a node is considered unreachable.
             * @param nodeAddress The address of the node to check.
             * @return true if the node is reachable, false otherwise.
             */
            bool isNodeReachable(uint16_t nodeAddress);

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

            void taskThread() override;

        };

    }

}

#endif