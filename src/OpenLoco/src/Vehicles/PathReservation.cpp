#include "Vehicles/PathReservation.h"
#include "Entities/EntityManager.h"
#include "Map/Track/Track.h"
#include "Map/Track/TrackData.h"
#include "Scenario/ScenarioManager.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/VehicleHead.h"
#include "Vehicles/RoutingManager.h"
#include "Vehicles/VehicleManager.h"
#include <unordered_map>

namespace OpenLoco::Vehicles::PathReservation
{
    struct Reservation
    {
        std::vector<Piece> pieces; // Route still to be chosen by the head
        // The piece the head has chosen to enter next. It isn't physically occupied until the head gets there so
        // it has to stay reserved, otherwise another train could be sent onto it.
        std::optional<Piece> chosen;
        CompanyId owner;
        uint8_t trackType;
    };

    struct TileRecord
    {
        uint8_t connectFlags;
        EntityId owner;
    };

    static std::unordered_map<EntityId, Reservation> _reservations;
    // The tiles covered by every reservation, rebuilt whenever a reservation changes or the tick changes
    static std::unordered_multimap<uint64_t, TileRecord> _tiles;
    // The tiles of the piece every train has chosen to enter next, used when looking for a free route.
    // Trains that never went through a path signal have no reservation but their next piece isn't occupied yet either.
    static std::unordered_multimap<uint64_t, TileRecord> _chosenTiles;
    static bool _chosenDirty = true;
    static uint32_t _chosenTick = 0xFFFFFFFFU;
    static bool _dirty = false;
    static uint32_t _validatedTick = 0xFFFFFFFFU;

    static uint64_t makeTileKey(const World::Pos3& pos)
    {
        return static_cast<uint64_t>(static_cast<uint16_t>(pos.x))
            | (static_cast<uint64_t>(static_cast<uint16_t>(pos.y)) << 16)
            | (static_cast<uint64_t>(static_cast<uint16_t>(pos.z)) << 32);
    }

    static TrackAndDirection::_TrackAndDirection getBasicTad(const Piece& piece)
    {
        const auto basicTad = piece.tad & World::Track::AdditionalTaDFlags::basicTaDMask;
        return TrackAndDirection::_TrackAndDirection((basicTad >> 3) & 0x3F, basicTad & 0x7);
    }

    // Calls f(tile position with height, connect flags) for every tile of the piece
    template<typename Func>
    static void forEachTile(const Piece& piece, Func&& f)
    {
        const auto tad = getBasicTad(piece);
        auto start = piece.pos;
        if (tad.isReversed())
        {
            const auto& trackSize = World::TrackData::getUnkTrack(tad._data);
            start += trackSize.pos;
            if (trackSize.rotationEnd < 12)
            {
                start -= World::Pos3{ World::kRotationOffset[trackSize.rotationEnd], 0 };
            }
        }

        for (const auto& trackPiece : World::TrackData::getTrackPiece(tad.id()))
        {
            const auto xy = World::Pos2{ start } + Math::Vector::rotate(World::Pos2{ trackPiece.x, trackPiece.y }, tad.cardinalDirection());
            f(World::Pos3{ xy, static_cast<int16_t>(start.z + trackPiece.z) }, trackPiece.connectFlags[tad.cardinalDirection()]);
        }
    }

    // Signals bounding the blocks the route passes through need to be told about the change
    static void refreshSignals(const CompanyId owner, const uint8_t trackType, const std::vector<Piece>& pieces)
    {
        for (auto i = 0U; i < pieces.size(); ++i)
        {
            if (i == 0 || (pieces[i - 1].tad & World::Track::AdditionalTaDFlags::hasSignal))
            {
                sub_4A2AD7(pieces[i].pos, getBasicTad(pieces[i]), owner, trackType);
            }
        }
    }

    // The piece the head has chosen to enter next but isn't physically on yet
    static std::optional<Piece> getChosenPiece(const VehicleHead& head)
    {
        RoutingManager::RingView ring(head.routingHandle);
        auto it = ring.begin();
        ++it;
        const auto routing = RoutingManager::getRouting(*it);
        if (routing == RoutingManager::kAllocatedButFreeRouting || routing == RoutingManager::kRoutingNull)
        {
            return std::nullopt;
        }
        const auto [nextPos, nextRotation] = World::Track::getTrackConnectionEnd(World::Pos3(head.tileX, head.tileY, head.tileBaseZ * World::kSmallZStep), head.trackAndDirection.track._data);
        return Piece{ nextPos, static_cast<uint16_t>(routing & World::Track::AdditionalTaDFlags::basicTaDWithSignalMask) };
    }

    // Only needed when a route is being reserved so unlike the reservations it is not kept up to date otherwise
    static void ensureChosenTilesUpToDate()
    {
        const uint32_t ticks = ScenarioManager::getScenarioTicks();
        if (!_chosenDirty && ticks == _chosenTick)
        {
            return;
        }
        _chosenDirty = false;
        _chosenTick = ticks;

        _chosenTiles.clear();
        for (auto* head : VehicleManager::VehicleList())
        {
            if (head->mode != TransportMode::rail || !head->isPlaced())
            {
                continue;
            }
            if (const auto chosen = getChosenPiece(*head))
            {
                forEachTile(*chosen, [&](const World::Pos3& tilePos, const uint8_t connectFlags) {
                    _chosenTiles.emplace(makeTileKey(tilePos), TileRecord{ connectFlags, head->id });
                });
            }
        }
    }

    static void ensureUpToDate()
    {
        if (_reservations.empty())
        {
            _tiles.clear();
            _dirty = false;
            return;
        }

        const uint32_t ticks = ScenarioManager::getScenarioTicks();
        if (!_dirty && ticks == _validatedTick)
        {
            return;
        }
        _validatedTick = ticks;
        _dirty = false;

        // Drop the routes of trains that no longer exist
        std::vector<Reservation> dropped;
        for (auto it = _reservations.begin(); it != _reservations.end();)
        {
            auto* head = EntityManager::get<VehicleHead>(it->first);
            if (head == nullptr || !head->isPlaced() || head->mode != TransportMode::rail)
            {
                dropped.push_back(std::move(it->second));
                it = _reservations.erase(it);
            }
            else
            {
                ++it;
            }
        }

        _tiles.clear();
        for (const auto& [headId, reservation] : _reservations)
        {
            const auto addPiece = [&, headId = headId](const Piece& piece) {
                forEachTile(piece, [&](const World::Pos3& tilePos, const uint8_t connectFlags) {
                    _tiles.emplace(makeTileKey(tilePos), TileRecord{ connectFlags, headId });
                });
            };
            for (const auto& piece : reservation.pieces)
            {
                addPiece(piece);
            }
            if (reservation.chosen)
            {
                addPiece(*reservation.chosen);
            }
        }

        // The tiles are up to date at this point so refreshing can safely query them
        for (const auto& reservation : dropped)
        {
            refreshSignals(reservation.owner, reservation.trackType, reservation.pieces);
        }
    }

    void reset()
    {
        _reservations.clear();
        _tiles.clear();
            _chosenTiles.clear();
            _chosenDirty = true;
            _dirty = false;
            _validatedTick = 0xFFFFFFFFU;
        }

    void set(VehicleHead& head, std::vector<Piece>&& pieces)
    {
        auto& reservation = _reservations[head.id];
        reservation.owner = head.owner;
        reservation.trackType = head.trackType;
        reservation.pieces = std::move(pieces);
        // The piece the head is on now is covered by the train being on it
        reservation.chosen.reset();
        _dirty = true;

        // refreshSignals can rebuild the table so work on a copy
        const auto copy = reservation.pieces;
        refreshSignals(head.owner, head.trackType, copy);
    }

    void clear(const VehicleHead& head)
    {
        const auto it = _reservations.find(head.id);
        if (it == _reservations.end())
        {
            return;
        }
        const auto dropped = std::move(it->second);
        _reservations.erase(it);
        _dirty = true;
        refreshSignals(dropped.owner, dropped.trackType, dropped.pieces);
    }

    bool isPieceFree(const VehicleHead& head, const Piece& piece)
    {
        ensureUpToDate();
        ensureChosenTilesUpToDate();

        bool isFree = true;
        const auto checkTiles = [&](const std::unordered_multimap<uint64_t, TileRecord>& tiles) {
            if (tiles.empty())
            {
                return;
            }
            forEachTile(piece, [&](const World::Pos3& tilePos, const uint8_t connectFlags) {
                const auto [begin, end] = tiles.equal_range(makeTileKey(tilePos));
                for (auto it = begin; it != end; ++it)
                {
                    if (it->second.owner != head.id && (it->second.connectFlags & connectFlags) != 0)
                    {
                        isFree = false;
                    }
                }
            });
        };
        checkTiles(_tiles);
        checkTiles(_chosenTiles);
        if (!isFree)
        {
            return false;
        }

        return !isTrackPieceOccupiedByOtherTrain(piece.pos, getBasicTad(piece), head.id);
    }

    bool isTileReserved(const World::Pos3& tileWorldPos, const uint8_t connectFlags)
    {
        ensureUpToDate();
        if (_tiles.empty())
        {
            return false;
        }

        const auto [begin, end] = _tiles.equal_range(makeTileKey(tileWorldPos));
        for (auto it = begin; it != end; ++it)
        {
            if ((it->second.connectFlags & connectFlags) != 0)
            {
                return true;
            }
        }
        return false;
    }

    std::optional<size_t> reservedConnectionIndex(const VehicleHead& head, const World::Pos3& pos, std::span<const uint16_t> connections)
    {
        const auto it = _reservations.find(head.id);
        if (it == _reservations.end() || it->second.pieces.empty())
        {
            return std::nullopt;
        }

        const auto& next = it->second.pieces.front();
        if (next.pos != pos)
        {
            return std::nullopt;
        }
        for (auto i = 0U; i < connections.size(); ++i)
        {
            if ((connections[i] & World::Track::AdditionalTaDFlags::basicTaDMask) == (next.tad & World::Track::AdditionalTaDFlags::basicTaDMask))
            {
                return i;
            }
        }
        return std::nullopt;
    }

    void onHeadEnter(const VehicleHead& head, const World::Pos3& pos, const uint16_t tad)
    {
        // Any train choosing a piece changes where trains are about to be
        _chosenDirty = true;
        if (!_reservations.empty())
        {
            _dirty = true;
        }

        const auto it = _reservations.find(head.id);
        if (it == _reservations.end())
        {
            return;
        }

        auto& reservation = it->second;
        auto& pieces = reservation.pieces;
        if (!pieces.empty()
            && pieces.front().pos == pos
            && (pieces.front().tad & World::Track::AdditionalTaDFlags::basicTaDMask) == (tad & World::Track::AdditionalTaDFlags::basicTaDMask))
        {
            // The previously chosen piece is physically occupied now. The new one is not yet so it stays reserved.
            reservation.chosen = pieces.front();
            pieces.erase(pieces.begin());
            _dirty = true;
            return;
        }

        // Either the route has been used up or the train left it so nothing more is reserved
        clear(head);
    }
}
