#ifndef EXVECTRNETWORK_H_
#define EXVECTRNETWORK_H_

#include "ExVectrNetwork/datalink.hpp"
#include "ExVectrNetwork/transport.hpp"
#include "ExVectrNetwork/network_node.hpp"

namespace VCTR
{

    /**
     * @brief   The ExVectrNetwork library
     * @note    Network structure similar to OSI model but with some differences:
     *              4. Transport    Uses Network node to send packets to other nodes. Takes care of packetizing, reassembling data, error checking and flow control.
     *              3. Network      Uses Datalink layer to send packets to other nodes. Takes care of routing and addressing.
     *              2. Data link    Uses Physical layer to gain access to a physical medium and send dataframes. Takes care of access timing and splitting data into frames.
     *              1. Physical     (UART, SPI, I2C, etc.) Uses the HAL digital IO interface.
     */
    namespace Net
    {

    }

} // namespace VCTR The EXVECTR Library

#endif