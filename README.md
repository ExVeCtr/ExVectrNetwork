# ExVectr Networking library
This library allows for networking ability to connect devices.
The network structure closely follows the ISO/OSI model.
Each layer has an interface class inside the interface folder. There are also general implementations inside the abstract folder. Theses are useful for getting a quick system running with minimal implementation work.
## Design:
The physical layer is the actual data transfer method. Usually connecting two system over a bus like SPI, UART or could also be radio system like LoRa modules. It is assumed that sending data over this bus will broadcast it to all other connections.

The Datalink layer controlles the access to the physical medium to prevent collisions and can possibly add some error checking/redundancy. This layer is required to make the interface with each physical layer the same in the context of data transfer.

The Network layer adds addressing and routing to the system. Network nodes are connected to a single datalink and network routers connect network nodes to connect network structures.

Finally the transport layer will packetize large data and possibly add network checks and connections for safe data transfer.

Additionally systems similar to DHCP (Automatic IP adressing) and DNS (Name to IP Address conversion) are planned to be added.
## Dependencies:
The ExVectr libraries below are required by this library to function.
- Core 
- HAL
## **This project is under initial development. Things will probably break.**