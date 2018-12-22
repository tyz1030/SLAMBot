#include <planning/frontiers.hpp>
#include <planning/motion_planner.hpp>
#include <common/grid_utils.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <queue>
#include <set>
#include <cassert>
#include <cmath>
#include <common/angle_functions.hpp>


frontier_t grow_frontier(Point<int> cell, const OccupancyGrid &map, std::set<Point<int>> &visitedFrontiers);

robot_path_t path_to_frontier(const frontier_t &frontier,
                              const pose_xyt_t &pose,
                              const OccupancyGrid &map,
                              const MotionPlanner &planner);

pose_xyt_t nearest_navigable_cell(pose_xyt_t pose,
                                  Point<float> desiredPosition,
                                  const OccupancyGrid &map,
                                  const MotionPlanner &planner);

pose_xyt_t search_to_nearest_free_space(Point<float> position, const OccupancyGrid &map, const MotionPlanner &planner);

double path_length(const robot_path_t &path);


std::vector<frontier_t> find_map_frontiers(const OccupancyGrid &map,
                                           const pose_xyt_t &robotPose,
                                           double minFrontierLength) {
    /*
    * To find frontiers, we use a connected components search in the occupancy grid. Each connected components consists
    * only of cells where is_frontier_cell returns true. We scan the grid until an unvisited frontier cell is
    * encountered, then we grow that frontier until all connected cells are found. We then continue scanning through the
    * grid. This algorithm can also perform very fast blob detection if you change is_frontier_cell to some other check
    * based on pixel color or another condition amongst pixels.
    */
    std::vector<frontier_t> frontiers;
    std::set<Point<int>> visitedCells;

    Point<int> robotCell = global_position_to_grid_cell(Point<float>(robotPose.x, robotPose.y), map);
    std::queue<Point<int>> cellQueue;
    cellQueue.push(robotCell);
    visitedCells.insert(robotCell);

    // Use a 4-way connected check for expanding through free space.
    const int kNumNeighbors = 4;
    const int xDeltas[] = {-1, 1, 0, 0};
    const int yDeltas[] = {0, 0, 1, -1};

    // Do a simple BFS to find all connected free space cells and thus avoid unreachable frontiers
    while (!cellQueue.empty()) {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();

        // Check each neighbor to see if it is also a frontier
        for (int n = 0; n < kNumNeighbors; ++n) {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);

            // If the cell has been visited or isn't in the map, then skip it
            if (visitedCells.find(neighbor) != visitedCells.end() || !map.isCellInGrid(neighbor.x, neighbor.y)) {
                continue;
            }
                // If it is a frontier cell, then grow that frontier
            else if (is_frontier_cell(neighbor.x, neighbor.y, map)) {
                frontier_t f = grow_frontier(neighbor, map, visitedCells);

                // If the frontier is large enough, then add it to the collection of map frontiers
                if (f.cells.size() * map.metersPerCell() >= minFrontierLength) {
                    frontiers.push_back(f);
                }
            }
                // If it is a free space cell, then keep growing the frontiers
            else if (map(neighbor.x, neighbor.y) < 0) {
                visitedCells.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }

    return frontiers;
}


robot_path_t plan_path_to_frontier(const std::vector<frontier_t> &frontiers,
                                   const pose_xyt_t &robotPose,
                                   const OccupancyGrid &map,
                                   const MotionPlanner &planner) {
    /*
    * NOTES:
    *   - If there's multiple frontiers, you'll need to decide which to drive to.
    *   - A frontier is a collection of cells, you'll need to decide which one to attempt to drive to.
    *   - The cells along the frontier might not be in the configuration space of the robot, so you won't necessarily
    *       be able to drive straight to a frontier cell, but will need to drive somewhere close.
    */

    std::cout << "Planning path to frontier\n";
    // strategy is to first find the closest frontier (closest distance is smallest)
    robot_path_t path;
    if (frontiers.empty()) {
        return path;
    }

    using Pt = Point<double>;
    // find the centroids of the frontiers
    std::vector<Pt> centroids;
    for (const auto& frontier : frontiers) {
        Pt centroid;
        centroid.x = 0;
        centroid.y = 0;
        for (const auto& point : frontier.cells) {
            centroid.x += point.x;
            centroid.y += point.y;
        }
        centroid.x /= frontier.cells.size();
        centroid.y /= frontier.cells.size();
        centroids.push_back(centroid);
    }

    std::cout << "Calculated centroids\n";


    // for each valid goal cell, find the closest one to the robot while also being close to centroid of frontiers
    const auto w = map.widthInCells();
    const auto h = map.heightInCells();
    const auto robotPoint = Point<double>(robotPose.x, robotPose.y);
    pose_xyt_t goal;
    auto lowestScore = 1e11;
    for (auto y = 0; y < h; ++y) {
        for (auto x = 0; x < w; ++x) {
            const auto point = Point<int>{x, y};
            const auto xyGlobal = grid_position_to_global_position(point, planner.obstacleDistances());
            pose_xyt_t goalCandidate;
            goalCandidate.x = xyGlobal.x;
            goalCandidate.y = xyGlobal.y;
            goalCandidate.theta = 0;

            if (!planner.isValidGoal(goalCandidate)) {
                continue;
            }

            const auto distToRobot = distance_between_points(xyGlobal, robotPoint);
            const auto requiredHeading = atan2(point.y - robotPoint.y, point.x - robotPoint.x);
            const auto headingDist = angle_diff_abs(requiredHeading, robotPose.theta);

            auto distsToCentroids = 0.0;
            for (const auto& centroid : centroids) {
                distsToCentroids += distance_between_points(xyGlobal, centroid);
            }

            const auto score = distToRobot + headingDist + 0.3 * distsToCentroids;
//                        std::cout << goalCandidate.x << ' ' << goalCandidate.y << " is a valid goal with d " << distToRobot << " dtheta " << headingDist << " centroids " << distsToCentroids << std::endl;

            if (score < lowestScore) {
                lowestScore = score;
                goal = goalCandidate;
            }
        }
    }

    if (lowestScore > 1e10) {
        std::cout << "No valid goals\n";
        return path;
    }

    std::cout << "Want to get to " << goal.x << ' ' << goal.y << "\n";

    // start with a guess close to it and increase tolerance with each failure
    int range = 0;
    auto foundPath = false;
    while (!foundPath) {
        for (auto i = -range; i <= range; ++i) {
            for (auto j = -range; j <= range; ++j) {
                auto testGoal = goal;
                testGoal.x = goal.x + i * map.metersPerCell();
                testGoal.y = goal.y + j * map.metersPerCell();
                // std::cout << "Test goal " << goal.x << ' ' << goal.y << std::endl;
                if (!planner.isValidGoal(testGoal)) {
                    continue;
                }

                path = planner.planPath(robotPose, testGoal);
                foundPath = path.path_length > 1;
                std::cout << "Test path ";
                for (const auto& pose : path.path) {
                    std::cout << "(" << pose.x << ' ' << pose.y << ") -> ";
                }
                std::cout << std::endl;

                if (foundPath) {
                    goal = testGoal;
                }
            }
        }
        // allow greater tolerance each iteration
        range += 1;
    }

    std::cout << "Accepted path ";
    for (const auto& pose : path.path) {
        std::cout << "(" << pose.x << ' ' << pose.y << ") -> ";
    }
    std::cout << std::endl;

    return path;
}


bool is_frontier_cell(int x, int y, const OccupancyGrid &map) {
    // A cell if a frontier if it has log-odds 0 and a neighbor has log-odds < 0

    // A cell must be in the grid and must have log-odds 0 to even be considered as a frontier
    if (!map.isCellInGrid(x, y) || (map(x, y) != 0)) {
        return false;
    }

    const int kNumNeighbors = 4;
    const int xDeltas[] = {-1, 1, 0, 0};
    const int yDeltas[] = {0, 0, 1, -1};

    for (int n = 0; n < kNumNeighbors; ++n) {
        // If any of the neighbors are free, then it's a frontier
        // Note that logOdds returns 0 for out-of-map cells, so no explicit check is needed.
        if (map.logOdds(x + xDeltas[n], y + yDeltas[n]) < 0) {
            return true;
        }
    }

    return false;
}


frontier_t grow_frontier(Point<int> cell, const OccupancyGrid &map, std::set<Point<int>> &visitedFrontiers) {
    // Every cell in cellQueue is assumed to be in visitedFrontiers as well
    std::queue<Point<int>> cellQueue;
    cellQueue.push(cell);
    visitedFrontiers.insert(cell);

    // Use an 8-way connected search for growing a frontier
    const int kNumNeighbors = 8;
    const int xDeltas[] = {-1, -1, -1, 1, 1, 1, 0, 0};
    const int yDeltas[] = {0, 1, -1, 0, 1, -1, 1, -1};

    frontier_t frontier;

    // Do a simple BFS to find all connected frontier cells to the starting cell
    while (!cellQueue.empty()) {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();

        // The frontier stores the global coordinate of the cells, so convert it first
        frontier.cells.push_back(grid_position_to_global_position(nextCell, map));

        // Check each neighbor to see if it is also a frontier
        for (int n = 0; n < kNumNeighbors; ++n) {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);
            if ((visitedFrontiers.find(neighbor) == visitedFrontiers.end())
                && (is_frontier_cell(neighbor.x, neighbor.y, map))) {
                visitedFrontiers.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }

    return frontier;
}