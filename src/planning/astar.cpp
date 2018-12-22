#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <common/grid_utils.hpp>
#include <queue>

using XY = Point<int>;

double distanceCost(const SearchParams &params, double distance) {
    if (distance >= params.maxDistanceWithCost) {
        return 0;
    }
    return std::pow(params.maxDistanceWithCost - distance, params.distanceCostExponent);
}

double euclideanDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

int cellIndex(int x, int y, int w) {
    return y * w + x;
}

int cellIndex(XY xy, int w) {
    return xy.y * w + xy.x;
}

robot_path_t search_for_path(pose_xyt_t start,
                             pose_xyt_t goal,
                             const ObstacleDistanceGrid &distances,
                             const SearchParams &params) {

//    std::cout << "Searching for path\n";

    const auto w = distances.widthInCells();
    const auto h = distances.heightInCells();
    // not sure if we need to consider orientation
    // search over grid space (a reasonable discretization since we have known distance to obstacles for each one)
    std::vector<bool> visited(w * h, false);
    std::vector<bool> visiting(w * h, false);

    // we'll use Euclidean distance heuristic + distance to obstacle cost
    // precompute heuristic for each cell
    std::vector<double> heurstic(w * h, 0);
    for (auto y = 0; y < h; ++y) {
        for (auto x = 0; x < w; ++x) {
            const auto xyGlobal = grid_position_to_global_position(Point<int>{x, y}, distances);
            heurstic[cellIndex(x, y, w)] =
                    distanceCost(params, distances(x, y)) + euclideanDistance(xyGlobal.x, xyGlobal.y, goal.x, goal.y);
        }
    }

    // cost to come
    // total cost is heuristic + total cost
    const auto inf = 1e10;  // not max because adding to max is not defined (can potentially wrap)
    std::vector<double> g(w * h, inf);
    auto startXY = global_position_to_grid_cell({start.x, start.y}, distances);
//    std::cout << "start " << start.x << ' ' << start.y << " -> " << startXY.x << " " << startXY.y << std::endl;
    // we don't start inside our grid...
    if (!distances.isCellInGrid(startXY.x, startXY.y)) {
//        std::cout << "rejecting start cell\n";
        return {};
    }

    g[cellIndex(startXY.x, startXY.y, w)] = 0;

    const auto edgeDist = distances.metersPerCell();
    const auto edgeDiagDist = edgeDist * sqrt(2);
    // for 8-neighbourhood
    const auto neighbours = std::vector<std::pair<XY, decltype(edgeDiagDist)>>{{{-1, -1}, edgeDiagDist},
                                                                               {{-1, 0},  edgeDist},
                                                                               {{-1, 1},  edgeDiagDist},
                                                                               {{0,  -1}, edgeDist},
                                                                               {{0,  1},  edgeDist},
                                                                               {{1,  -1}, edgeDiagDist},
                                                                               {{1,  0},  edgeDist},
                                                                               {{1,  1},  edgeDiagDist}};

    // who the predecessor of this node is
    std::vector<XY> parents(w * h);

    auto cmp = [&](const XY &a, const XY &b) {
        const auto aIndex = cellIndex(a.x, a.y, w);
        const auto bIndex = cellIndex(b.x, b.y, w);
        // we'd like the lowest total cost on top
        // cmp should return true if a < b
        // but q.top() is actually the tail end, so return false if a should be popped before b
        return (g[aIndex] + heurstic[aIndex]) > (g[bIndex] + heurstic[bIndex]);
    };
    std::priority_queue<XY, std::vector<XY>, decltype(cmp)> q(cmp);
    // initialize with the starting element
    q.push(startXY);
    parents[cellIndex(startXY, w)] = startXY;

    // start algorithm
    const auto goalXY = global_position_to_grid_cell({goal.x, goal.y}, distances);

    auto foundSolution = false;
    while (!q.empty()) {
        const auto xy = q.top();
//        std::cout << q.size() << ' ';
//        std::cout << "pop " << xy.x << ' ' << xy.y << std::endl;
        q.pop();

        if (xy.x == goalXY.x && xy.y == goalXY.y) {
            foundSolution = true;
            break;
        }

        // assume cost is monotonic so we don't have to explore again
        const auto currentIndex = cellIndex(xy.x, xy.y, w);
//        std::cout << "current index " << currentIndex << " size " << visited.size() << std::endl;
        // could potentially be visiting it twice (not sifting up/down so could have duplicates)
        if (visited[currentIndex]) {
            continue;
        }
//        std::cout << "not visited\n";

        visited[currentIndex] = true;

        for (const auto &edge : neighbours) {
            const auto dxy = edge.first;
            const auto nxy = XY{xy.x + dxy.x, xy.y + dxy.y};

            // out of bounds
            if (!distances.isCellInGrid(nxy.x, nxy.y)) {
                continue;
            }

//            std::cout << "neighbour " << nxy.x << ' ' << nxy.y << std::endl;

            // would lead to a collision by going to the cell
            if (distances(nxy.x, nxy.y) - params.minDistanceToObstacle < 1e-5) {
                continue;
            }

            // already finished exploring
            const auto neighbourIndex = cellIndex(nxy.x, nxy.y, w);
//            std::cout << "neighbour index " << neighbourIndex << std::endl;
            if (visited[neighbourIndex]) {
                continue;
            }

//            std::cout << "this cost " << g[currentIndex] << " neighbour cost " << g[neighbourIndex] << std::endl;
            const auto distThroughThisPath = g[currentIndex] + edge.second;
            // node needs update
            if (distThroughThisPath < g[neighbourIndex]) {
                parents[neighbourIndex] = xy;
                g[neighbourIndex] = distThroughThisPath;
                q.push(nxy);
            }
        }
    }
//    std::cout << "Done\n";

    if (!foundSolution) {
        return {};
    }

    // backtrace parents to get path
    robot_path_t path;
    path.utime = start.utime;
    // insert in reverse order and then we reverse the path
    path.path.push_back(goal);
    auto lastXY = parents[cellIndex(goalXY, w)];
//    std::cout << "(" << goalXY.x << ' ' << goalXY.y << ") <- ";
    while (lastXY != startXY) {
//        std::cout << "(" << lastXY.x << ' ' << lastXY.y << ") <- ";
        // TODO necessary to compute theta? we're going to turn in place to face the next waypoint anyway
        pose_xyt_t next{};
        const auto lastXYInGlobal = grid_position_to_global_position(Point<double>{(double)lastXY.x,(double)lastXY.y}, distances);
        next.x = lastXYInGlobal.x;
        next.y = lastXYInGlobal.y;
        next.theta = 0;
        path.path.push_back(next);
        lastXY = parents[cellIndex(lastXY, w)];
    }
//    std::cout << "(" << startXY.x << ' ' << startXY.y << ")\n";
    path.path.push_back(start);
    std::reverse(path.path.begin(), path.path.end());
    path.path_length = path.path.size();
    return path;
}
