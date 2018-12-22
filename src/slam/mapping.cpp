#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds), _firstPose(true)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    if (_firstPose) {
        _firstPose = false;
        _lastPose = pose;
    }

    _lastPose = pose;
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    const auto mls = MovingLaserScan(scan, _lastPose, pose);

    for (int i = 0; i < (int)mls.size(); ++i) {
        const auto& ray = mls[i];

        ///convert pose to start cell
        const auto x0 = map.mapToGridX(ray.origin.x);
        const auto y0 = map.mapToGridY(ray.origin.y);

        // project final point as the sensor can finally read
        const auto x1 = map.mapToGridX(ray.origin.x + ray.range * cos(ray.theta));
        const auto y1 = map.mapToGridY(ray.origin.y + ray.range * sin(ray.theta));
        //std::cout << x0 << ' ' << y0 << ' ' << ray.origin.x << ' ' << ray.origin.y << std::endl;
        //std::cout << x1 << ' ' << y1 << ' ' << ray.origin.x + ray.range * cos(ray.theta) << ' ' << ray.origin.y + ray.range * sin(ray.theta) << std::endl;
        ///Breshenham’s Algorithm
        const auto dx = abs(x1 - x0);
        const auto dy = abs(y1 - y0);
        const auto sx = (x0 < x1) ? 1 : -1;
        const auto sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;
        auto x = x0;
        auto y = y0;
        while((x != x1) || (y != y1)) {
            //occupancy grid
            if (!map.isCellInGrid(x, y)) {
                break;
            }

            auto& odds = map(x, y);
            if (odds > std::numeric_limits<CellOdds>::lowest()+1){
                odds -= 1;
            } else {
                odds = std::numeric_limits<CellOdds>::lowest();
            }

            const auto e2 = 2*err;
            if (e2 >= -dy){
                err -= dy;
                x += sx;
            }
            if (e2 <= dx){
                err += dx;
                y += sy;
            }
        }

/**
// another approach with unit vector
        const auto veclen = 0.5;
        const float x0 = ray.origin.x;
        const float y0 = ray.origin.y;
        const float x1 = ray.origin.x + ray.range * cos(ray.theta);
        const float y1 = ray.origin.y + ray.range * sin(ray.theta);

        const auto x1_grid = map.mapToGridX(x1);
        const auto y1_grid = map.mapToGridY(y1);

        const auto dx = x1 - x0;
        const auto dy = y1 - y0;

        const auto unitdx = veclen * dx / ray.range;
        const auto unitdy = veclen * dy / ray.range;

        float x = x0;
        float y = y0;
        
        auto x_grid = map.mapToGridX(x);
        auto y_grid = map.mapToGridY(y);
        printf ("hello\n");
        while((x_grid != x1_grid) || (y_grid != y1_grid)) {
            //occupancy grid
            if (!map.isCellInGrid(x_grid, y_grid)) {
                break;
            }

            auto& odds = map(x_grid, y_grid);
            if (odds > std::numeric_limits<CellOdds>::lowest()+1){
                odds -= 1;
            } else {
                odds = std::numeric_limits<CellOdds>::lowest();
            }

            x += unitdx;
            y += unitdy;
            x_grid = map.mapToGridX(x);
            y_grid = map.mapToGridY(y);
        }
        auto& odds = map(x_grid, y_grid);
*/
 
        //occupancy grid
        auto& odds = map(x, y);
        if (odds < std::numeric_limits<CellOdds>::max()-4) {
            odds += 4;
        } else {
            odds = std::numeric_limits<CellOdds>::max();
        }
            
    }
}
