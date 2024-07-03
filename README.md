# By The Way Logistics
By the way logistics - a social platform for shipment transportation

Project carried out as part of the course "Team Project" and the Team Project Conference (KPZ) in the summer semester 2022/23 by PWr students together with the company Comarch.

The goal of the project was to design a parcel/package transport platform based on multiple users transporting parcels e.g. for daily transportation (to work, university, etc.). The project group was tasked with designing a logistics container (closed, sealed, size-standardized, GPS-tracked along the lines of a shipping container scaled to the project's needs, of course) and an application in which users could be both (and at the same time) couriers and parcel receivers.

![Component communication diagram](./resources/comm_diag.jpg)

The original repository can be found [here](https://github.com/dominicus28/BTWL).

### Project structure
The repository consists of two branches, one for each project component:
- [device](https://github.com/3p3v/btwl/tree/device) - contains the implementation of the embedded device that uses ESP32,
- [backend](https://github.com/3p3v/btwl/tree/backend) - contains the API implemented in Spring Boot.

## Device
The aim of this subproject was to create a device (sealed inside the specially prepared ***container***) that could be used for monitoring the transportation of the goods, including:
- checking temperature and humidity inside the container,
- careless handling of the container and its contents (by using accelerometer),
- sending GPS coordinates of its current location,
- detecting unauthorized opening of the container and send a warning to the server

The device also had to:
- allow user to open the container,
- send all data mentioned above in for of a JSON, that could be understood by the server.

The data was sent by the device by a GPRS network (using a SIM800 module).

### Tools
1) Programming language
   - C/C++,
2) ESP32 programming environment:
   - ESP-IDF.

### Libraries
The following libraries were used:
1) [I2Cdev](https://github.com/jrowberg/i2cdevlib), including the MPU6050 library:
   - MPU6050 support,
2) [minmea]([minmea](https://github.com/kosma/minmea)):
   - NMEA0183 message parser,
3) [cJSON](https://github.com/DaveGamble/cJSON):
   - JSON format parser, used to create messages sent to and received from the server,
4) Catch2 v3:
   - unit tests.

Libraries for other used modules (SIM800l, SHT30, NEO-6m) were written exclusively for this project as other libraries did not provide expected functionality or had a license other than MIT (and could not be used for any potentially commercial product).

## Backend
The purpose of this API was to provide communication between users and transport containers. 
It handles queries sent from users and, based on them, returns information stored in the database about available orders, parcels sent by the user and parcels received by the user, their status, etc.
All the logic of the system is handled by API, the application only sends requests, the transport container executes the issued commands. 

The backend follows REST API architectural style.
All requests use JSON format.