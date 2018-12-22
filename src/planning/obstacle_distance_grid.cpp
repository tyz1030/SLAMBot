#include <queue>
#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>
#include <iomanip>


ObstacleDistanceGrid::ObstacleDistanceGrid(void)
        : width_(100), height_(100), metersPerCell_(0.05f), cellsPerMeter_(20.0f) {
}


void ObstacleDistanceGrid::setDistances(const OccupancyGrid &map) {
    using XY = std::pair<int, int>;
    using FillQueue = std::queue<XY>;

    resetGrid(map);

    // for 8-neighbourhood
    const auto neighbours = std::vector<XY>{{-1, -1},
                                            {-1, 0},
                                            {-1, 1},
                                            {0,  -1},
                                            {0,  1},
                                            {1,  -1},
                                            {1,  0},
                                            {1,  1}};

    // flood fill from every obstacle grid
    // stop propagation when we encounter an obstacle or a distance measure lower than our distance to it
    // assume everything is infinitely far from obstacles
    std::fill(cells_.begin(), cells_.end(), std::numeric_limits<float>::max());

    auto atLeastOneObstacle = false;
    for (int y = 0; y < map.heightInCells(); ++y) {
        for (int x = 0; x < map.widthInCells(); ++x) {
            // unknown (not an obstacle and not free)
            if (map(x, y) == 0) {
                distance(x, y) = 0;
                continue;
            }
//            std::cout << std::setw(5) << x << std::setw(5) << y << std::setw(5) << (int)map(x,y) << std::endl;
            if (map(x, y) < 0) {
                continue;
            }
//            std::cout << "Checking this XY pair" << std::endl;

            // not an obstacle or unknown

            atLeastOneObstacle = true;
            distance(x, y) = 0;

            // start flood filling
            FillQueue q;
            q.emplace(x, y);

            std::vector<bool> visited(cells_.size(), false);

            while (!q.empty()) {
                const auto xy = q.front();
                q.pop();


                for (const auto dxy : neighbours) {
                    const auto nx = xy.first + dxy.first;
                    const auto ny = xy.second + dxy.second;


                    // out of bounds
                    if (!isCellInGrid(nx,ny)) {
                        continue;
                    }

                    // already visited
                    const auto ij = cellIndex(nx, ny);
                    if (visited[ij]) {
                        continue;
                    }

                    // is an obstacle or unknown don't need to explore
                    if (map(nx,ny) >= 0) {
                        continue;
                    }


                    // calculate unscaled distance from this obstacle to it (center to center)
                    // we scale everything afterwards to save processing cycles
                    // squared distance in cell lengths
                    const auto dist = (nx-x)*(nx-x) + (ny-y)*(ny-y);
                    auto& storedDist = distance(nx, ny);

                    // don't need to update if our distance to it from another obstacle is closer
                    // 0 is a special value meaning we haven't explored it yet so we explicitly ignore it
                    if (storedDist != 0 && dist - storedDist > 1e-5) {
                        continue;
                    }

                    // else we update it and put it onto our queue to explore its neighbours
                    storedDist = dist;
                    q.emplace(nx, ny);
                    visited[ij] = true;
                }

            }
        }
    }


    // need to set all elements to 0 according to test
    if (atLeastOneObstacle) {
        // scale all distances to meters
        for (int y = 0; y < map.heightInCells(); ++y) {
            for (int x = 0; x < map.widthInCells(); ++x) {
                distance(x, y) = sqrt(distance(x, y)) * metersPerCell_;
//                distance(x, y) = sqrt(distance(x, y));
            }
        }
    }
}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const {
    if (((x >= 0) && (x < width_)) && ((y >= 0) && (y < height_))) {
        return true;
    } else {
        return false;
    }
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid &map) {
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();

    // If the grid is already the correct size, nothing needs to be done
    if ((width_ == map.widthInCells()) && (height_ == map.heightInCells())) {
        return;
    }

    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();

    cells_.resize(width_ * height_);
}
