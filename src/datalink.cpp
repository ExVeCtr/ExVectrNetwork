#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrCore/print.hpp"
#include "ExVectrCore/list.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrHAL/digital_io.hpp"

#include "ExVectrNetwork/datalink.hpp"

/**
 * The datalink works by first sending a block command to the physical layer. This will block the physical layer for a set amount of time. This way only one node can send at a time.
 * Then the datalink will send the data in frames of the maximum size the physical layer supports and then send a free command to release the physical layer.
 */

namespace VCTR
{

    namespace Net
    {

        Datalink::Datalink(HAL::DigitalIO &physicalLayerDevice, Core::Scheduler &scheduler) : Task_Periodic("Datalink", 1000 * Core::MILLISECONDS)
        {
            physicalLayer_ = &physicalLayerDevice;
            scheduler.addTask(*this);
        }

        void Datalink::dataframeReceiveFunc(const Core::List<uint8_t> &item)
        {

            VRBS_MSG("Received %d bytes from topic to send. Pointer %d \n", item.size(), this);

            auto len = item.size();
            if (len > dataLinkMaxFrameLength)
            {
                LOG_MSG("Datalink: Max frame length exceeded. Failure.\n");
                return; // Max frame length exceeded. Failure.
            }
            if (len > transmitBuffer_.sizeMax() - transmitBuffer_.size() - 1)
            {
                LOG_MSG("Datalink: Buffer overflow. Failure.\n");
                return; // Buffer overflow case. Failure.
            }

            transmitBuffer_.placeBack(len);
            for (size_t i = 0; i < len; i++)
                transmitBuffer_.placeBack(item[i]);
        }

        void Datalink::setPhysicalReleaseTimeout(int64_t time)
        {
            physicalReleaseTime_ = time;
        }

        void Datalink::taskInit()
        {

            // Lock system for the case that we connect to an already running system.
            transmitting_ = false;
            physicalBlocked_ = true;
            physicalBlockTimestamp_ = Core::NOW() - 1 * Core::SECONDS; // Listen for an extra second.
        }

        void Datalink::taskThread()
        {

            VRBS_MSG("Datalink thread running. Pointer %d. Time: %f\n", this, Core::NOWSeconds());

            // Read from physical and check status
            auto readLen = physicalLayer_->readable();
            if (readLen > 0)
            {

                VRBS_MSG("Reading %d bytes. Receiving is %d\n", readLen, receiving_);

                if (!receiving_)
                {

                    uint8_t data;
                    physicalLayer_->readByte(data);

                    switch (PhysicalHeader(data))
                    {
                    case PhysicalHeader::BLOCK: // Physical is now in use by another node.
                        transmitting_ = false;
                        physicalBlocked_ = true;
                        physicalBlockTimestamp_ = Core::NOW();

                        VRBS_MSG("Header is block.\n");

                        break;

                    case PhysicalHeader::DATA: // Received data from channel. Block usage.
                        // VRBS_MSG("Header data with %d bytes.\n", buffer[1]);
                        transmitting_ = false;
                        physicalBlocked_ = true;
                        receiving_ = true;
                        physicalBlockTimestamp_ = Core::NOW();

                        physicalLayer_->readByte(data);
                        numBytesReceive_ = data;

                        VRBS_MSG("Header is data with %d bytes.\n", numBytesReceive_);

                        break;

                    case PhysicalHeader::FREE: // Channel has been freed up for use. If data was received, then publish it.
                        physicalBlocked_ = false;
                        receiving_ = false;

                        if (receiveBuffer_.size() > 0)
                        {
                            VRBS_MSG("Publishing %d bytes. This: %d \n", receiveBuffer_.size(), this);
                            receiveTopic_.publish(receiveBuffer_);
                            receiveBuffer_.clear();
                        }

                        VRBS_MSG("Header is free.\n");

                        break;

                    default:
                        VRBS_MSG("Header unknown.\n");
                        break;
                    }
                }

                if (receiving_)
                {

                    physicalBlockTimestamp_ = Core::NOW(); // Reset timeout

                    uint8_t size = numBytesReceive_;
                    if (size > readLen)
                        size = readLen;

                    if (size > receiveBuffer_.sizeMax() - receiveBuffer_.size())
                    {

                        LOG_MSG("Datalink: Buffer overflow. Failure.\n");
                    }
                    else
                    {
                        uint8_t buffer[size];
                        physicalLayer_->readData(buffer, size);

                        for (size_t i = 0; i < size; i++)
                            receiveBuffer_.placeBack(buffer[i]);
                        numBytesReceive_ -= size;

                        if (numBytesReceive_ == 0)
                            receiving_ = false;

                        VRBS_MSG("Received %d bytes.\n", size);
                    }
                }
            }

            // Check if timeout was reached for physical access. In this case we can reset access.
            if (Core::NOW() - physicalBlockTimestamp_ > physicalReleaseTime_)
            {
                physicalBlockTimestamp_ = Core::END_OF_TIME; // END_OF_TIME to stop this from triggering again.
                physicalBlocked_ = false;
            }

            // If physical is unblocked, then try to gain access if there is data to send.
            if (!physicalBlocked_ && (transmitBuffer_.size() > 0 || transmitting_))
            {

                auto writeLen = physicalLayer_->writable();
                if (transmitting_ && numBytesTransmit_ == 0 && writeLen > 0)
                { // Free medium if nothing more to write.

                    VRBS_MSG("Freeing medium!\n");

                    transmitting_ = false;
                    physicalLayer_->writeByte(uint8_t(PhysicalHeader::FREE));
                }
                else if (!transmitting_ && writeLen > 0)
                { // Gain access to medium

                    numBytesTransmit_ = transmitBuffer_[0]; // Number of bytes to transmit is first byte.
                    transmitBuffer_.removeFront();
                    transmitting_ = true;

                    VRBS_MSG("Ready to send: %d bytes. Blocking medium.\n", numBytesTransmit_);

                    physicalLayer_->writeByte(uint8_t(PhysicalHeader::BLOCK));
                }
                else if (transmitting_ && writeLen > 2)
                {

                    size_t sendLen = numBytesTransmit_;
                    if (sendLen > 250)
                        sendLen = 250;
                    if (sendLen > writeLen - 2)
                        sendLen = writeLen - 2;

                    uint8_t buffer[sendLen + 2];
                    buffer[0] = uint8_t(PhysicalHeader::DATA);
                    buffer[1] = sendLen;
                    for (uint8_t i = 0; i < sendLen; i++)
                        buffer[i + 2] = transmitBuffer_[i];
                    transmitBuffer_.removeFront(sendLen);

                    VRBS_MSG("Sending: %d\n", sendLen);

                    physicalLayer_->writeData(buffer, sendLen + 2);

                    numBytesTransmit_ -= sendLen;
                }
            }
        }

        void Datalink::taskCheck()
        {

            if (transmitBuffer_.size() > 0 || (transmitting_ && physicalLayer_->writable() > 0) || physicalLayer_->readable() > 0)
            {
                setRelease(Core::NOW());
            }
        }

    }

}