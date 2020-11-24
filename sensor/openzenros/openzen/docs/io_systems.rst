.. _io-system-label:

##########
IO Systems
##########

IO Systems are connecting to sensors via various data transport layers. If you use the
``ZenClient::listSensorsAsync`` method, all IO systems will list the sensor they were
able to auto-discover and you can connect to a sensor without knowing any additional details
on the IO system.

In some case you may want to connect directly to a sensor because the sensor name is known.
In this case, you need to also provide the name of the IO system the sensor is connected on.

Furthermore, some IO systems don't support auto-discovery of sensors and they can only be used
with the ``ZenClient::obtainSensorByName`` method.

SiLabs USB Express
==================
LP-Research sensors which are configured to use the USB-mode can be connected via this IO system
on Windows. The advantage is that the Baudrate does not need to be configured and the sensor can
be configured via string name. This name is always the same, no matter on which USB port the sensor
is connected or which other peripheral devices are connected.

=======================     ============
Name in OpenZen             SiUsb
Supported Platforms         Windows only
Supports auto-discovery     yes
=======================     ============

Example to obtain a USB connected sensor which has the name lpmscu2000573.

.. code-block:: cpp

    auto sensorPair = client.obtainSensorByName("SiUsb", "lpmscu2000573");

Windows COM Port
================
LP-Research sensors which are configured to use the COM port mode can be connected via this IO system
on Windows. The baudrate needs to be provided with the call to ``obtainSensorByName`` and the sensor name
is the COM-port named assigned to the sensor by Windows. This name can be different on different systems,
depending how much other COM-Port devices are connected.

=======================     =============
Name in OpenZen             WindowsDevice
Supported Platforms         Windows only
Supports auto-discovery     yes
=======================     =============

Example to obtain a COM-Port connected sensor which is connected on the Windows COM-Port COM12 using
a baudrate of 115200 bits per second.

.. code-block:: cpp

    auto sensorPair = client.obtainSensorByName("WindowsDevice", "COM12", 115200);

Bluetooth 
=========
The Bluetooth IO system can be used to connect to bluetooth sensors like the LP-Research LPMS-B2 sensor.
To be able to connect to any bluetooth sensor, it first needs to be paired via the operating system's
device manager. Then it can be auto-discovered by OpenZen or directly connected via the bluetooth address
of the sensor.

=======================     ===================
Name in OpenZen             Bluetooth
Supported Platforms         Windows, Linux, Mac
Supports auto-discovery     yes
=======================     ===================

Example to obtain a bluetooth sensor which has the bluetooth address 00:11:22:33:FF:EE:

.. code-block:: cpp

    auto sensorPair = client.obtainSensorByName("Bluetooth", "00:11:22:33:FF:EE");

Bluetooth Low-Energy
====================
The Bluetooth Low-Energy IO system can be used to connect to bluetooth sensors like the LP-Research LPMS-B2 sensor
via the low-enery mode of Bluetooth.
To be able to connect to any bluetooth sensor, it first needs to be paired via the operating system's
device manager. Then it can be auto-discovered by OpenZen or directly connected via the bluetooth address
of the sensor.

=======================     ===================
Name in OpenZen             Ble
Supported Platforms         Windows, Linux, Mac
Supports auto-discovery     yes
=======================     ===================

Example to obtain a bluetooth sensor which has the bluetooth address 00:11:22:33:FF:EE:

.. code-block:: cpp

    auto sensorPair = client.obtainSensorByName("Ble", "00:11:22:33:FF:EE");

Linux Device
============
Allows to connect to a sensor which is connected via the USB-mode on Linux systems. It is the
equivalent of the SiUsb IO system on Linux in that it needs not baud rate configuration and only
the device's name to connect.

Sensors can only be connected on Linux if the user running the OpenZen process has read and write access to the
serial devices of the system. To allow this, the users needs to be added to the dialout group. This can be
done with this command:

.. code-block:: bash

    sudo adduser <username> dialout

=======================     ===================
Name in OpenZen             LinuxDevice
Supported Platforms         Linux
Supports auto-discovery     yes
=======================     ===================

Example to obtain a bluetooth sensor which has the name lpmscu2000573

.. code-block:: cpp

    auto sensorPair = client.obtainSensorByName("LinuxDevice", "lpmscu2000573");

Network Streaming with ZeroMQ
=============================
This interface system allows to receive sensor data from another OpenZen instance over the network. Therefore,
it does not connect to any local sensor but opens a network connection. Still, the received events are provided
via the OpenZen event loop to the user and therefore appear like regular events from a local sensor.
The ZeroMQ interface has some limitations in the features its provides for sensor access. For example, it does
not support to start or stop streaming of the sensor or to reconfigure any settings on the sensor. This needs to
be done by the OpenZen instance which is physically connected to the sensor. Furthermore, ZeroMQ  can not be used
to query the components connected to the sensor.

On the machine where the sensor is physically connected to:

.. code-block:: cpp

    // connect to the sensor via the physical interface
    auto sensorPair = client.obtainSensorByName("SiUsb", "lpmscu2000573");
    auto& sensor = sensorPair.second;
    // publish sensor data via TCP to all hosts on port 8877
    sensor.publishEvents("tcp://*:8877");

On the machine which should receive the sensor data over the network:

.. code-block:: cpp

    // connect to the remote instance of OpenZen running on the machine with the IP address 192.168.1.34
    auto sensorPair = client.obtainSensorByName("ZeroMQ", "tcp://192.168.1.34:8877");

    // now events received over the network can be queried via the normal OpenZen
    // waitForNextEvent() call
    const auto pair = client.get().waitForNextEvent();

=======================     ===================
Name in OpenZen             ZeroMQ
Supported Platforms         Linux, Windows, Mac
Supports auto-discovery     no
=======================     ===================
