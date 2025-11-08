/**
 * @copyright 2018 Infineon Technologies
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 */

#include "BoardSerial.hpp"
#include "BridgeSerial.hpp"

#include <common/Logger.hpp>
#include <platform/BoardListProtocol.hpp>
#include <platform/templates/searchBoardFunction.hpp>

namespace
{
    constexpr const uint32_t defaultBaudrate  = 921600;
    constexpr const uint32_t fallbackBaudrate = 1000000;
}

std::unique_ptr<BoardDescriptor> BoardSerial::searchBoard(const char port[], BoardData::const_iterator begin, BoardData::const_iterator end)
{
    LOG(DEBUG) << "Looking for board on " << port << " ...";

    // Boards with the KitProg3 debugger do not support the default baud rate, see RADARFW-20.
    // Since the Strata protocol uses a CRC in its communication we can easily identify baud rates
    // by trial-and-error that are fully supported.
    try
    {
        return searchBoardFunctionBridge<BridgeSerial>(begin, end, port, defaultBaudrate);
    }
    catch (const EException &)
    {
        try
        {
            return searchBoardFunctionBridge<BridgeSerial>(begin, end, port, fallbackBaudrate);
        }
        catch (const EException &)
        {
            throw;
        }
    }
}

std::unique_ptr<BoardInstance> BoardSerial::createBoardInstance(const char port[])
{
    return searchBoard(port, BoardListProtocol::begin, BoardListProtocol::end)->createBoardInstance();
}
