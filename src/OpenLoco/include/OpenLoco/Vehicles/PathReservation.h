#pragma once

#include "Types.hpp"
#include <OpenLoco/Engine/World.hpp>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace OpenLoco::Vehicles
{
    struct VehicleHead;

    // Routes that trains have reserved ahead of them with path signals.
    // This is runtime state only, it is not saved and is cleared when a game is loaded.
    namespace PathReservation
    {
        struct Piece
        {
            World::Pos3 pos; // Position the piece is entered at, same convention as track connections
            uint16_t tad;    // Basic track and direction with the hasSignal flag
        };

        void reset();

        // Replaces the route reserved by the train
        void set(VehicleHead& head, std::vector<Piece>&& pieces);
        // Drops the route reserved by the train
        void clear(const VehicleHead& head);

        // True if no other train has reserved or is standing on the piece
        bool isPieceFree(const VehicleHead& head, const Piece& piece);

        // True if any train has reserved a track tile at this height that shares a connection with connectFlags
        bool isTileReserved(const World::Pos3& tileWorldPos, uint8_t connectFlags);

        // If the next piece of the train's reserved route is one of the connections at pos returns its index
        std::optional<size_t> reservedConnectionIndex(const VehicleHead& head, const World::Pos3& pos, std::span<const uint16_t> connections);

        // The head is about to enter the piece, the reserved route is advanced or dropped if the train left it
        void onHeadEnter(const VehicleHead& head, const World::Pos3& pos, uint16_t tad);
    }
}
