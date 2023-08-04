#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrCore/list.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/print.hpp"

#include "ExVectrHAL/digital_io.hpp"

#include "ExVectrNetwork/datalink.hpp"

namespace VCTR
{

    namespace Net
    {

        Datalink::Datalink(HAL::DigitalIO &physicalLayerDevice, Core::Scheduler &scheduler) : Task_Periodic("Datalink", 100*Core::MILLISECONDS)
        {
            physicalLayer_ = &physicalLayerDevice;
            transmitSubr_.setCallback(this, &Datalink::dataframeReceiveFunc);
            scheduler.addTask(*this);
        }

        Core::Topic<Core::List<uint8_t>> &Datalink::getReceiveTopic()
        {
            return receiveTopic_;
        }

        void Datalink::setTransmitTopic(Core::Topic<Core::List<uint8_t>> &transmitTopic)
        {
            transmitSubr_.subscribe(transmitTopic);
        }

        void Datalink::removeTransmitTopic(Core::Topic<Core::List<uint8_t>> &transmitTopic)
        {
            transmitSubr_.unsubcribe(transmitTopic);
        }

        void Datalink::removeTransmitTopic()
        {
            transmitSubr_.unsubcribe();
        }

        void Datalink::dataframeReceiveFunc(const Core::List<uint8_t> &item)
        {

            auto len = item.size();
            if (len > dataLinkMaxFrameLength) return; //Max frame length exceeded. Failure.
            if (len > transmitBuffer_.sizeMax() - transmitBuffer_.size() - 1) return; //Buffer overflow case. Failure.

            transmitBuffer_.placeBack(len);
            for (size_t i = 0; i < len; i++) transmitBuffer_.placeBack(item[i]);

        }

        void Datalink::setPhysicalReleaseTimeout(int64_t time) {
            physicalReleaseTime_ = time; 
        }

        void Datalink::taskInit() {
            
            //Lock system for the case that we connect to an already runnign system.
            transmitting_ = false;
            physicalBlocked_ = true;
            physicalBlockTimestamp_ = Core::NOW() - 1*Core::SECONDS; //Listen for an extra second.

        }

        void Datalink::taskThread() {
            
            //Read from physical and check status
            auto readLen = physicalLayer_->readable();
            if (readLen > 0) {

                Core::printM("Read %d bytes.\n", readLen);

                if (!receiving_) {

                    uint8_t data;
                    physicalLayer_->readByte(data);
                    
                    switch (PhysicalHeader(data))
                    {
                    case PhysicalHeader::BLOCK : //Physical is now in use by another node.
                        transmitting_ = false;
                        physicalBlocked_ = true;
                        physicalBlockTimestamp_ = Core::NOW();

                        Core::printM("Header block.\n");

                        break;

                    case PhysicalHeader::DATA : //Received data from channel. Block usage.
                        //Core::printM("Header data with %d bytes.\n", buffer[1]);
                        transmitting_ = false;
                        physicalBlocked_ = true;
                        receiving_ = true;
                        physicalBlockTimestamp_ = Core::NOW();

                        physicalLayer_->readByte(data);
                        numBytesReceive_ = data;

                        Core::printM("Header data with %d bytes.\n", numBytesReceive_);

                        break;

                    case PhysicalHeader::FREE : //Channel has been freed up for use. If data was received, then publish it.
                        physicalBlocked_ = false;

                        if (receiveBuffer_.size() > 0) {
                            receiveTopic_.publish(receiveBuffer_);
                            receiveBuffer_.clear();
                        }

                        Core::printM("Header free.\n");

                        break; 
                    
                    default:
                        Core::printM("Header unknown.\n");
                        break;
                    }

                }

                if (receiving_) {

                    uint8_t size = numBytesReceive_;
                    if (size > readLen) size = readLen;

                    uint8_t buffer[size];
                    physicalLayer_->readData(buffer, size);

                    for (size_t i = 0; i < size; i++) receiveBuffer_.placeBack(buffer[i]);
                    numBytesReceive_ -= size;

                    if (numBytesReceive_ == 0) receiving_ = false;

                    Core::printM("Received %d bytes.\n", size);

                }

            }
            
            //Check if timeout was reached for physical access. In this case we can reset access.
            if (Core::NOW() - physicalBlockTimestamp_ > physicalReleaseTime_) {
                physicalBlockTimestamp_ = Core::END_OF_TIME; //END_OF_TIME to stop this from triggering again.
                physicalBlocked_ = false;
            }

            //If physical is unblocked, then try to gain access if there is data to send.
            if (!physicalBlocked_ && (transmitBuffer_.size() > 0 || transmitting_)) {

                auto writeLen = physicalLayer_->writable();
                if (writeLen > 2) {

                    if (transmitting_ && numBytesTransmit_ == 0) { //Free medium if nothing more to write.
                        
                        //Core::printM("Freeing medium!\n");

                        transmitting_ = false;
                        physicalLayer_->writeByte(uint8_t(PhysicalHeader::FREE));

                    } else if (!transmitting_) { //Gain access to medium

                        numBytesTransmit_ = transmitBuffer_[0]; //Number of bytes to transmit is first byte.
                        transmitBuffer_.removeFront();
                        transmitting_ = true;

                        //Core::printM("Ready to send: %d bytes. Blocking medium...\n", numBytesTransmit_);

                        physicalLayer_->writeByte(uint8_t(PhysicalHeader::BLOCK));

                    } else {
                        
                        if (writeLen > numBytesTransmit_) writeLen = numBytesTransmit_;
                        if (writeLen > 250) writeLen = 250;

                        uint8_t buffer[writeLen + 2];
                        buffer[0] = uint8_t(PhysicalHeader::DATA);
                        buffer[1] = writeLen;
                        for (uint8_t i = 0; i < writeLen; i++) buffer[i + 2] = transmitBuffer_[i];
                        transmitBuffer_.removeFront(writeLen);

                        //Core::printM("Sending: %d\n", writeLen);

                        physicalLayer_->writeData(buffer, writeLen + 2);

                        numBytesTransmit_ -= writeLen;

                    }
                
                }

            }

        }

        void Datalink::taskCheck() {

            if ((transmitting_ && physicalLayer_->writable()) || physicalLayer_->readable() > 0 || transmitBuffer_.size() > 0) {
                setRelease(Core::NOW());
            }

        }

    }

}