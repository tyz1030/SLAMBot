#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>
#include <common/math.h>
#include <fstream>

// values from rplidar1 spec sheets
// take std to be something times the resolution
SensorModel::SensorModel() : _rangeStd(0.002 * 300), _rangeMax(12), _hitObstacle(0.01), _detectNothing(0.01),
                             _detectRandom(0.1 / _rangeMax), _wallOdds(20) {
    std::ifstream f{"sensor_model.cfg"};
    f >> _rangeStd >> _hitObstacle >> _detectNothing >> _detectRandom >> _wallOdds;
    _rangeStd *= 0.002;
    _detectRandom /= _rangeMax;
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map) {
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    const auto mls = MovingLaserScan(scan, sample.parent_pose, sample.pose);
    // for each scan, ray cast onto the occupancy grid and compute likelihood as random sampling from a distribution
    // distribution will be a gaussian with the mean being the expected distance to a wall

    // unnormalized
    double likelihood = 1;
    for (int i = 0; i < (int) mls.size(); ++i) {
        // don't need to use every scan for now
        if (i % 3 != 0) {
            continue;
        }

        const auto ray = mls.at(i);
        const auto trueDist = rayCast(ray, map);
        const auto dist = static_cast<double>(ray.range);

        const auto l = rayLikelihood(trueDist, dist);
        // likelihood of scan is the product of the likelihood of individual ray measurements
        likelihood *= l;
    }

    // see page 157 for a picture of the mixed density
    return likelihood;
}

double SensorModel::rayCast(const adjusted_ray_t& ray, const OccupancyGrid& map) const {
    // Breshenham's algorithm
    const auto x0 = map.mapToGridX(ray.origin.x);
    const auto y0 = map.mapToGridY(ray.origin.y);
    // project final point as the sensor can finally read
    const auto x1 = map.mapToGridX(ray.origin.x + _rangeMax * cos(ray.theta));
    const auto y1 = map.mapToGridY(ray.origin.y + _rangeMax * sin(ray.theta));
    const auto dx = abs(x1 - x0);
    const auto dy = abs(y1 - y0);
    const auto sx = (x0 < x1) ? 1 : -1;
    const auto sy = (y0 < y1) ? 1 : -1;
    auto err = dx - dy;
    auto x = x0;
    auto y = y0;

    while (x != x1 || y != y1) {
        // if we're outside the map assume there's no wall and that we should read max reading
        if (!map.isCellInGrid(x, y)) {
            break;
        }
        // do something such as update the odds of the grid at (x,y)
        if (map(x, y) >= _wallOdds) {
            // distance from origin to center of this pixel
            const auto ddx = ray.origin.x - map.gridToMapX(x);
            const auto ddy = ray.origin.y - map.gridToMapY(y);
            return sqrt(ddx * ddx + ddy * ddy);
        }
        // move to next integer location
        const auto e2 = 2 * err;
        if (e2 >= -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y += sy;
        }
    }
    // beyond what our sensor can read, so should give max reading
    return _rangeMax;
}

double SensorModel::rayLikelihood(double trueDist, double dist) const {
    double rangeLikelihood = 0;
    // mixture of different probabilities
    if (dist >= 0 && dist <= _rangeMax) {
        // nominal gaussian hitting the expected distance
        rangeLikelihood += normalPdf(dist, trueDist, _rangeStd);

        // unexpected objects cause ranges to be shorter than true distance and decreases with range
        if (dist <= trueDist) {
            rangeLikelihood += _hitObstacle * std::exp(-_hitObstacle * dist);
        }

        // random measurements uniformly distributed
        rangeLikelihood += _detectRandom;
    }

    // failures from sensing black objects (absorbs all light, or too much ambient light)
    if (fabs(dist - _rangeMax) < 0.1) {
        rangeLikelihood += _detectNothing;
    }
    return rangeLikelihood;
}

void SensorModel::printPdf(double trueDist, double granularity, std::ostream& out) const {
    double range = 0;
    while (range <= 1.2 * _rangeMax) {
        out << range << ", " << rayLikelihood(trueDist, range) << '\n';
        range += granularity;
    }
}
