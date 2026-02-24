#include "ExVectrCore/cyclic_checksum.hpp"
#include "ExVectrCore/list.hpp"
#include "ExVectrCore/list_static.hpp"
#include "ExVectrCore/print.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrNetwork/datalink/DatalinkI.hpp"
#include "ExVectrNetwork/network/NetworkHeader.hpp"
#include "ExVectrNetwork/network/NetworkNode.hpp"

namespace VCTR::network::network {
NetworkNode::NetworkNode(uint16_t nodeAddress, int64_t disconnectTimeout)
    : NetworkI(nodeAddress),
      Task_Periodic("NetworkNode", 100 * Core::MILLISECONDS) // 10Hz
{
  timeoutInterval_ = disconnectTimeout;
  sendInterval_ =
      timeoutInterval_ /
      10; // Send a heartbeat packet every 1/5 of the timeout interval.

  Core::getSystemScheduler().addTask(*this);
}

NetworkNode::NetworkNode(uint16_t nodeAddress, datalink::DatalinkI &datalink,
                         int64_t disconnectTimeout)
    : NetworkNode(nodeAddress, disconnectTimeout) {
  addDatalink(datalink);
}

// NetworkNode::~NetworkNode()
//{
// }

void NetworkNode::addDatalink(datalink::DatalinkI &datalink) {
  datalinks_.append(&datalink);
  datalink.addReceiveHandler(
      [this](const DataPacket &dataframe) { receivePacket(dataframe); });
}

bool NetworkNode::isNodeReachable(uint16_t nodeAddress) {
  for (size_t i = 0; i < nodeList_.size(); i++) {
    if (nodeList_[i].nodeAddress == nodeAddress) {
      return true;
    }
  }
  return false;
}

void NetworkNode::taskThread() {

  if (Core::NOW() - lastSend_ > sendInterval_) {
    lastSend_ = Core::NOW();
    NetworkPacketHeader header;
    header.type = NetworkPacketType::HEARTBEAT;
    header.hops = 0;
    header.dstAddress = 0xFFFF; // Broadcast to all nodes.
    header.srcAddress = nodeAddress_;
    sendPacket(header);
  }

  // Check if any nodes are unreachable
  for (size_t i = 0; i < nodeList_.size(); i++) {
    if (Core::NOW() - nodeList_[i].lastSeen > timeoutInterval_) {
      nodeList_.removeAtIndex(i);
      i--;
    }
  }
}

void NetworkNode::sendPacket(const NetworkPacketHeader &header,
                             const Core::ListArray<uint8_t> &payload) {

  if (payload.size() == 0) {
    LOG_MSG("Packet empty! \n");
    return;
  }

  auto headerSend = header;
  headerSend.srcAddress = nodeAddress_;
  // packetSend.type = NetworkPacketType::DATA;

  if (nodeAddress_ ==
      header.dstAddress) { // If this packet is for this node, publish it
                           // directly to the receive topic.
    packetReceiveHandlers_.callHandlers(header, payload);
    return;
  }

  // LOG_MSG("Packet input size: %d, Packet copy size: %d\n",
  // packet.payload.size(), packetSend.payload.size());
  DataPacket packetSend;
  headerSend.addHeader(packetSend);

  for (size_t i = 0; i < datalinks_.size(); i++) {
    datalinks_[i]->transmitDataframe(packetSend);
  }
  lastSend_ = Core::NOW(); // Update the last send time.
}

size_t NetworkNode::getMaxPacketSize() const {
  size_t maxSize = 0;
  for (size_t i = 0; i < datalinks_.size(); i++) {
    size_t datalinkMaxSize = datalinks_[i]->getMaxPacketSize();
    if (datalinkMaxSize > maxSize) {
      maxSize = datalinkMaxSize;
    }
  }
  return maxSize;
}

void NetworkNode::receivePacket(const DataPacket &data) {

  // Convert the data to a packet
  NetworkPacketHeader header;
  if (!header.fromPacket(data)) {
    VRBS_MSG("Failed to unpack packet! \n");
    return;
  }

  // Update the reachable nodes list
  bool found = false;
  for (size_t i = 0; i < nodeList_.size(); i++) {
    if (nodeList_[i].nodeAddress == header.srcAddress) {
      nodeList_[i].lastSeen = Core::NOW();
      found = true;
      break;
    }
  }
  if (!found) // // If the node is not in the list, add it
  {
    NodeInfo nodeInfo;
    nodeInfo.nodeAddress = header.srcAddress;
    nodeInfo.lastSeen = Core::NOW();
    nodeList_.appendIfNotInListArray(nodeInfo);
  }

  if (header.type == NetworkPacketType::HEARTBEAT) {
    return; // Ignore heartbeat packets.
  }

  // Check if the node is for us and publish it to the receive topic
  if (header.dstAddress == nodeAddress_ ||
      header.dstAddress ==
          UINT16_MAX) // If this packet is for this node, publish it directly to
                      // the receive topic.
  {
    // Decrement hops if not zero
    if (header.hops > 0)
      header.hops--;

    // Send packet to receive topic (Network -> Transport)
    auto payload = data.payload;
    payload.popDiscard(header.getHeaderSize());
    packetReceiveHandlers_.callHandlers(header, payload);
  }
}
} // namespace VCTR::network::network