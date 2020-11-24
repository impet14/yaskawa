//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZen, under the MIT License.
// See https://bitbucket.org/lpresearch/openzen/src/master/LICENSE for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

#include "io/bluetooth/BluetoothDeviceHandler.h"

#include "utility/bluetooth-serial-port/BluetoothException.h"

#include <spdlog/spdlog.h>

namespace zen
{
    BluetoothDeviceHandler::BluetoothDeviceHandler(std::string_view address) noexcept
        : m_address(address), m_buffer(256, std::byte(0))
    {}

    BluetoothDeviceHandler::~BluetoothDeviceHandler() {
        if (m_device) {
            m_device->Close();
        }
    }

    ZenSensorInitError BluetoothDeviceHandler::initialize() noexcept
    {
        try {
            m_device = std::unique_ptr<BTSerialPortBinding>(BTSerialPortBinding::Create(std::string(m_address), 1));
            m_device->Connect();
            return ZenSensorInitError_None;
        }
        catch(BluetoothException & be) {
            spdlog::error("Cannot connect to Bluetooth sensor: {0}", be.what());
            return ZenSensorInitError_ConnectFailed;
        }
    }

    nonstd::expected<std::vector<std::byte>, ZenError> BluetoothDeviceHandler::read() noexcept
    {
        auto nBytes = m_device->Read(reinterpret_cast<char *>(m_buffer.data()), m_buffer.size());
        SPDLOG_DEBUG("Read {0} bytes trom bluetooth interface", nBytes);
        if (nBytes == 0)
            return nonstd::make_unexpected(ZenError_Io_ReadFailed);
        m_buffer.resize(nBytes);
        // copy return buffer
        return m_buffer;
    }

    ZenError BluetoothDeviceHandler::send(gsl::span<const std::byte> data) noexcept
    {
        try {
            m_device->Write(reinterpret_cast<const char*>(data.data()),
                static_cast<int>(data.size()));
            SPDLOG_DEBUG("Wrote {0} bytes to bluetooth interface", data.size());
        } catch(BluetoothException &) {
            return ZenError_Io_SendFailed;
        }
        return ZenError_None;
    }

    bool BluetoothDeviceHandler::equals(std::string_view address) const noexcept
    {
        return m_address == address;
    }
}
