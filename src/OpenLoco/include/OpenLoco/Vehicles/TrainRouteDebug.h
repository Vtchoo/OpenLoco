#pragma once

#include <OpenLoco/Engine/World.hpp>
#include <cstdint>
#include <vector>

namespace OpenLoco::Vehicles
{
    struct VehicleHead;

    // A track piece as seen by the pathfinder, position and basic track and direction
    struct TrackRoutePiece
    {
        World::Pos3 pos;
        uint16_t tad;
    };

    // Marks where a route piece is entered and which way the train is heading
    struct TrackRouteArrow
    {
        World::Pos3 pos;    // Tile position and height the piece is entered at
        uint8_t direction;  // Heading as a track rotation (0-3 cardinal, 12-15 diagonal)
    };

    // Arrows along the route the pathfinder currently picks for the train to reach its current order
    std::vector<TrackRouteArrow> calculateTrainRouteArrows(VehicleHead& head);

    // Debug overlay that draws the route of the train in the front most vehicle window as arrows
    namespace TrainRouteDebug
    {
        bool isEnabled();
        void setEnabled(bool enabled);
        // Call once per UI update, refreshes the route when needed
        void update();
        // Arrows to paint for the tile at tilePos (world coordinates) or nullptr if there are none
        const std::vector<TrackRouteArrow>* getArrowsAt(const World::Pos2& tilePos);
    }
}
