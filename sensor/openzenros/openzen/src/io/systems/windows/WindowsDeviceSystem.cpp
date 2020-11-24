//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZen, under the MIT License.
// See https://bitbucket.org/lpresearch/openzen/src/master/LICENSE for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

#include "io/systems/windows/WindowsDeviceSystem.h"

#include "io/interfaces/windows/WindowsDeviceInterface.h"

#include "EnumerateSerialPorts.h"

namespace zen
{
    ZenError WindowsDeviceSystem::listDevices(std::vector<ZenSensorDesc>& outDevices)
    {
        std::vector<std::string> serialPorts;
        if (!EnumerateSerialPorts(serialPorts))
            return ZenError_Device_ListingFailed;

        for (auto it = serialPorts.begin(); it != serialPorts.end(); ++it) {
            ZenSensorDesc desc;

            const std::string name("COM PORT #" + it->substr(3));
            std::memcpy(desc.name, name.c_str(), name.size());
            desc.name[name.size()] = '\0';

            desc.serialNumber[0] = '\0';
            std::memcpy(desc.ioType, WindowsDeviceSystem::KEY, sizeof(WindowsDeviceSystem::KEY));

            const std::string filename("\\\\.\\" + *it);
            std::memcpy(desc.identifier, filename.c_str(), filename.size());
            desc.identifier[filename.size()] = '\0';

            desc.baudRate = 921600;
            outDevices.emplace_back(desc);
        }
        return ZenError_None;
    }

    nonstd::expected<std::unique_ptr<IIoInterface>, ZenSensorInitError> WindowsDeviceSystem::obtain(const ZenSensorDesc& desc, IIoDataSubscriber& subscriber) noexcept
    {
        auto handle = ::CreateFileA(desc.identifier, GENERIC_READ | GENERIC_WRITE,
            0, nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
        if (handle == INVALID_HANDLE_VALUE)
            return nonstd::make_unexpected(ZenSensorInitError_InvalidAddress);

        // Create overlapped objects for asynchronous communication
        OVERLAPPED ioReader{ 0 };
        ioReader.hEvent = ::CreateEventA(nullptr, false, false, nullptr);
        if (!ioReader.hEvent)
            return nonstd::make_unexpected(ZenSensorInitError_IoFailed);

        OVERLAPPED ioWriter{ 0 };
        ioWriter.hEvent = ::CreateEventA(nullptr, false, false, nullptr);
        if (!ioWriter.hEvent)
            return nonstd::make_unexpected(ZenSensorInitError_IoFailed);

        auto ioInterface = std::make_unique<WindowsDeviceInterface>(subscriber, desc.identifier, handle, ioReader, ioWriter);

        DCB config;
        if (!::GetCommState(handle, &config))
            return nonstd::make_unexpected(ZenSensorInitError_IoFailed);

        config.fOutxCtsFlow = false;
        config.fOutxDsrFlow = false;
        config.fDtrControl = DTR_CONTROL_ENABLE;
        config.fOutX = false;
        config.fInX = false;
        config.fRtsControl = RTS_CONTROL_ENABLE;
        config.ByteSize = 8;
        config.Parity = NOPARITY;
        config.StopBits = ONESTOPBIT;

        if (!::SetCommState(handle, &config))
            return nonstd::make_unexpected(ZenSensorInitError_IoFailed);

        COMMTIMEOUTS timeoutConfig;
        timeoutConfig.ReadIntervalTimeout = 1;
        timeoutConfig.ReadTotalTimeoutMultiplier = 1;
        timeoutConfig.ReadTotalTimeoutConstant = 1;
        timeoutConfig.WriteTotalTimeoutMultiplier = 1;
        timeoutConfig.WriteTotalTimeoutConstant = 1;

        if (!::SetCommTimeouts(handle, &timeoutConfig))
            return nonstd::make_unexpected(ZenSensorInitError_IoFailed);

        //constexpr static unsigned int DEFAULT_BAUDRATE = CBR_115200;
        //if (auto error = ioInterface->setBaudRate(DEFAULT_BAUDRATE))
        //    return nonstd::make_unexpected(ZenSensorInitError_IoFailed);

        return ioInterface;
    }
}
