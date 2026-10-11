#include "Vehicles/TrainRouteDebug.h"
#include "Entities/EntityManager.h"
#include "Ui/WindowManager.h"
#include "Vehicles/VehicleHead.h"
#include "ViewportManager.h"
#include <algorithm>
#include <optional>
#include <unordered_map>

namespace OpenLoco::Vehicles::TrainRouteDebug
{
    // The route is refreshed at least this often (in UI updates) to pick up signal and track changes
    constexpr uint32_t kRefreshInterval = 40;

    // What the route depends on, the route is recalculated when it changes
    struct Signature
    {
        EntityId head;
        uint16_t currentOrder;
        int32_t tileX;
        int32_t tileY;
        int32_t tileBaseZ;
        uint16_t tad;

        bool operator==(const Signature& rhs) const = default;
    };

    static bool _enabled = false;
    static uint32_t _updatesSinceRefresh = 0;
    static std::optional<Signature> _signature;
    static std::vector<TrackRouteArrow> _arrows;
    // Arrows by the tile (world position) they are painted on
    static std::unordered_map<uint32_t, std::vector<TrackRouteArrow>> _arrowsByTile;

    static uint32_t makeTileKey(const World::Pos2& pos)
    {
        return static_cast<uint32_t>(static_cast<uint16_t>(pos.x)) | (static_cast<uint32_t>(static_cast<uint16_t>(pos.y)) << 16);
    }

    static void invalidateArrows(const std::vector<TrackRouteArrow>& arrows)
    {
        for (const auto& arrow : arrows)
        {
            Ui::ViewportManager::invalidate(World::Pos2{ arrow.pos.x, arrow.pos.y }, arrow.pos.z, arrow.pos.z + 48);
        }
    }

    static bool isSameRoute(const std::vector<TrackRouteArrow>& lhs, const std::vector<TrackRouteArrow>& rhs)
    {
        return std::equal(lhs.begin(), lhs.end(), rhs.begin(), rhs.end(), [](const auto& a, const auto& b) {
            return a.pos == b.pos && a.direction == b.direction;
        });
    }

    static void setRoute(std::vector<TrackRouteArrow>&& arrows)
    {
        if (isSameRoute(arrows, _arrows))
        {
            return;
        }

        // Viewports only repaint invalidated areas so both the old and new route must be redrawn
        invalidateArrows(_arrows);
        _arrows = std::move(arrows);
        _arrowsByTile.clear();
        for (const auto& arrow : _arrows)
        {
            _arrowsByTile[makeTileKey(World::Pos2{ arrow.pos.x, arrow.pos.y })].push_back(arrow);
        }
        invalidateArrows(_arrows);
    }

    // The train of the front most vehicle window, so only one route is ever shown
    static VehicleHead* getFocusedTrainHead()
    {
        for (auto i = Ui::WindowManager::count(); i != 0; --i)
        {
            auto* window = Ui::WindowManager::get(i - 1);
            if (window->type != Ui::WindowType::vehicle)
            {
                continue;
            }
            auto* head = EntityManager::get<VehicleHead>(EntityId(window->number));
            if (head == nullptr || head->mode != TransportMode::rail)
            {
                return nullptr;
            }
            return head;
        }
        return nullptr;
    }

    bool isEnabled()
    {
        return _enabled;
    }

    void setEnabled(bool enabled)
    {
        _enabled = enabled;
        _signature.reset();
        if (!enabled)
        {
            setRoute({});
        }
    }

    void update()
    {
        if (!_enabled)
        {
            return;
        }

        auto* head = getFocusedTrainHead();
        if (head == nullptr)
        {
            _signature.reset();
            setRoute({});
            return;
        }

        const Signature signature{
            head->id,
            head->currentOrder,
            head->tileX,
            head->tileY,
            head->tileBaseZ,
            head->trackAndDirection.track._data,
        };
        ++_updatesSinceRefresh;
        if (_signature == signature && _updatesSinceRefresh < kRefreshInterval)
        {
            return;
        }
        _signature = signature;
        _updatesSinceRefresh = 0;
        setRoute(calculateTrainRouteArrows(*head));
    }

    const std::vector<TrackRouteArrow>* getArrowsAt(const World::Pos2& tilePos)
    {
        if (_arrowsByTile.empty())
        {
            return nullptr;
        }
        const auto it = _arrowsByTile.find(makeTileKey(tilePos));
        return it == _arrowsByTile.end() ? nullptr : &it->second;
    }
}
