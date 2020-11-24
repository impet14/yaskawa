//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZen, under the MIT License.
// See https://bitbucket.org/lpresearch/openzen/src/master/LICENSE for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

// request always the C++14 interface of OpenZen, even if the compiler
// can support he C++17 interface
#define OPENZEN_CXX14
#include "OpenZen.h"

#include <iostream>

using namespace zen;

int main(int argc, char* argv[])
{
    // enable resonable log output for OpenZen
    ZenSetLogLevel(ZenLogLevel_Info);

    // create OpenZen Clien
    auto clientPair = make_client();
    auto& clientError = clientPair.first;
    auto& client = clientPair.second;

    if (clientError) {
        std::cout << "Cannot create OpenZen client" << std::endl;
        return clientError;
    }

    // connect to sensor on IO System by the sensor name
    // auto sensorPair = client.obtainSensorByName("SiUsb", "lpmscu2000573");
    auto sensorPair = client.obtainSensorByName("Bluetooth", "00:04:3E:9E:00:5A");
    auto& obtainError = sensorPair.first;
    auto& sensor = sensorPair.second;
    if (obtainError)
    {
        std::cout << "Cannot connect to sensor" << std::endl;
        client.close();
        return obtainError;
    }

    // check that the sensor has an IMU component
    auto imuPair = sensor.getAnyComponentOfType(g_zenSensorType_Imu);
    auto& hasImu = imuPair.first;
    auto imu = imuPair.second;

    if (!hasImu)
    {
        std::cout << "Connected sensor has no IMU" << std::endl;
        client.close();
        return ZenError_WrongSensorType;
    }

    // readout up to 200 samples from the IMU
    for (int i = 0; i < 200; i++) {
        auto event = client.waitForNextEvent();
        if (event.second.component.handle == imu.component().handle) {
            std::cout << "> Acceleration: \t x = " << event.second.data.imuData.a[0]
                << "\t y = " << event.second.data.imuData.a[1]
                << "\t z = " << event.second.data.imuData.a[2] << std::endl;
            std::cout << "> Gyro: \t\t x = " << event.second.data.imuData.g[0]
                << "\t y = " << event.second.data.imuData.g[1]
                << "\t z = " << event.second.data.imuData.g[2] << std::endl;
        }
    }

    client.close();
    std::cout << "Sensor connection closed" << std::endl;
    return 0;
}
