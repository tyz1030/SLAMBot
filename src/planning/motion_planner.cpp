#include <planning/motion_planner.hpp>
#include <planning/astar.hpp>
#include <common/grid_utils.hpp>
#include <common/timestamp.h>
#include <lcmtypes/robot_path_t.hpp>
#include <cmath>
#include <iomanip>


MotionPlanner::MotionPlanner(const MotionPlannerParams &params)
        : params_(params) {
    setParams(params);
}


MotionPlanner::MotionPlanner(const MotionPlannerParams &params, const SearchParams &searchParams)
        : params_(params), searchParams_(searchParams) {
}


robot_path_t MotionPlanner::planPath(const pose_xyt_t &start,
                                     const pose_xyt_t &goal,
                                     const SearchParams &searchParams) const {

    std::cout << "Plan path from " << start.x << ' ' << start.y << " to " << goal.x << ' ' << goal.y << std::endl;
    // If the goal isn't valid, then no path can actually exist
    if (!isValidGoal(goal)) {
        robot_path_t failedPath;
        failedPath.utime = utime_now();
        failedPath.path_length = 1;
        failedPath.path.push_back(start);

        std::cout << "INFO: path rejected due to invalid goal\n";

        return failedPath;
    }

    auto path = search_for_path(start, goal, distances_, searchParams);
    // return invalid paths immediately
    if (path.path_length <= 1) {
        return path;
    }
    
    // smooth path to remove waypoints where the next point is the same slope as the current waypoint
    auto waypoint = path.path.begin() + 1;
    auto lastWaypoint = path.path.begin();
    while (waypoint + 1 != path.path.end()) {
        // if the last slope is the same as to the next one
        const auto dxLast = waypoint->x - lastWaypoint->x;
        const auto dyLast = waypoint->y - lastWaypoint->y;
        auto nextWaypoint = waypoint + 1;
        const auto dx = nextWaypoint->x - waypoint->x;
        const auto dy = nextWaypoint->y - waypoint->y;

        const auto slopeLast = atan2(dyLast, dxLast);
        const auto slope = atan2(dy, dx);
        if (fabs(slope - slopeLast) < 0.01) {
            waypoint = path.path.erase(waypoint);
        } else {
            lastWaypoint = waypoint;
            waypoint = nextWaypoint;
        }
    }

    // smooth again by removing waypoints as long as the subpaths aren't too close to an obstacle
    waypoint = path.path.begin() + 1;
    lastWaypoint = path.path.begin();
    while (waypoint + 1 != path.path.end()) {
        const auto start = global_position_to_grid_cell(Point<double>{lastWaypoint->x, lastWaypoint->y}, distances_);
        auto nextWaypoint = waypoint + 1;
        const auto end = global_position_to_grid_cell(Point<double>{nextWaypoint->x, nextWaypoint->y}, distances_);

        const auto x0 = start.x;
        const auto y0 = start.y;
        // project final point as the sensor can finally read
        const auto x1 = end.x;
        const auto y1 = end.y; 

        const auto dx = abs(x1 - x0);
        const auto dy = abs(y1 - y0);
        const auto sx = (x0 < x1) ? 1 : -1;
        const auto sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;
        auto x = x0;
        auto y = y0;

        bool intermediateSafe = true;
        while((x != x1) || (y != y1)) {
            //occupancy grid
            if (!distances_.isCellInGrid(x, y)) {
                break;
            }

            if (distances_(x,y) <= searchParams.minDistanceToObstacle) {
                intermediateSafe = false;
                std::cout << "intermediate not safe " << x <<  ' ' << y << std::endl;
                break;
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

        // remove if we can get from previous to next safely without using this waypoint
        if (intermediateSafe) {
            waypoint = path.path.erase(waypoint);
            std::cout << "removing intermediate waypoint\n";
        } else {
            lastWaypoint = waypoint;
            waypoint = nextWaypoint;
        }
    }

    // std::cout << "path with waypoints\n";
    // for (const auto& pose : path.path) {
    //     std::cout << pose.x << ' ' << pose.y << std::endl;
    // }
    // Otherwise, use A* to find the path
    return path;
}


robot_path_t MotionPlanner::planPath(const pose_xyt_t &start, const pose_xyt_t &goal) const {
    return planPath(start, goal, searchParams_);
}


bool MotionPlanner::isValidGoal(const pose_xyt_t &goal) const {
    float dx = goal.x - prev_goal.x, dy = goal.y - prev_goal.y;
    float distanceFromPrev = std::sqrt(dx * dx + dy * dy);

    //if there's more than 1 frontier, don't go to a target that is within a robot diameter of the current pose
    if (num_frontiers != 1 && distanceFromPrev < 2 * searchParams_.minDistanceToObstacle) {
//        std::cout << "Too close to current pose\n";
        return false;
    }

    auto goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances_);

//    std::cout << "Goal cell " << goalCell.x << ' ' << goalCell.y << std::endl;
    // A valid goal is in the grid
    if (distances_.isCellInGrid(goalCell.x, goalCell.y)) {
        // std::cout << distances_.width_ << std::endl;

//        std::cout << "goal is in the grid with distance " << distances_(goalCell.x, goalCell.y) << std::endl;
        // And is far enough from obstacles that the robot can physically occupy the space
        // Add an extra cell to account for discretization error and make motion a little safer by not trying to
        // completely snuggle up against the walls in the motion plan
        const auto farAwayFromObstacle = distances_(goalCell.x, goalCell.y) > params_.robotRadius;
        if (!farAwayFromObstacle) {
//            std::cout << "Too close " << distances_(goalCell.x, goalCell.y) << std::endl;
        }
        return farAwayFromObstacle;
    }
//    std::cout << "Not in the map\n";
    // A goal must be in the map for the robot to reach it
    return false;
}


bool MotionPlanner::isPathSafe(const robot_path_t &path) const {

    ///////////// TODO: Implement your test for a safe path here //////////////////

    return true;
}


void MotionPlanner::setMap(const OccupancyGrid &map) {
    distances_.setDistances(map);

//    std::cout.precision(2);
//    for (int y = 0; y < map.heightInCells(); ++y) {
//        for (int x = 0; x < map.widthInCells(); ++x) {
//            std::cout << std::setw(5) << (int)map(x, y);
//        }
//        std::cout << std::endl;
//    }
//
//    std::cout << "\n\n\n" << std::endl;

    // print map
//    std::cout.precision(2);
//    for (int y = 0; y < map.heightInCells(); ++y) {
//        for (int x = 0; x < map.widthInCells(); ++x) {
//            std::cout << std::setw(5);
//            if (map(x, y) >= 0) {
//                std::cout << ((map(x, y) == 0) ? 'U' : 'X');
//            } else {
//                const auto d = distances_(x,y);
//                if (d > 1e10) {
//                    std::cout << "inf";
//                } else {
//                    std::cout << d;
//                }
//            }
//        }
//        std::cout << std::endl;
//    }
}


void MotionPlanner::setParams(const MotionPlannerParams &params) {
    searchParams_.minDistanceToObstacle = params_.robotRadius;
    searchParams_.maxDistanceWithCost = 10.0 * searchParams_.minDistanceToObstacle;
    searchParams_.distanceCostExponent = 1.0;
}
