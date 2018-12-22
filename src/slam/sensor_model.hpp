#ifndef SLAM_SENSOR_MODEL_HPP
#define SLAM_SENSOR_MODEL_HPP

#include <ostream>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>

class  lidar_t;
struct particle_t;

/**
* SensorModel implement a sensor model for computing the likelihood that a laser scan was measured from a
* provided pose, give a map of the environment.
* 
* A sensor model is compute the unnormalized likelihood of a particle in the proposal distribution.
*
* To use the SensorModel, a single method exists:
*
*   - double likelihood(const particle_t& particle, const lidar_t& scan, const OccupancyGrid& map)
*
* likelihood() computes the likelihood of the provided particle, given the most recent laser scan and map estimate.
*/
class SensorModel
{
public:

    /**
    * Constructor for SensorModel.
    */
    SensorModel(void);

    /**
    * likelihood computes the likelihood of the provided particle, given the most recent laser scan and map estimate.
    * 
    * \param    particle            Particle for which the log-likelihood will be calculated
    * \param    scan                Laser scan to use for estimating log-likelihood
    * \param    map                 Current map of the environment
    * \return   Likelihood of the particle given the current map and laser scan.
    */
    double likelihood(const particle_t& particle, const lidar_t& scan, const OccupancyGrid& map);
    double rayLikelihood(double trueDist, double dist) const;
    double rayCast(const adjusted_ray_t& ray, const OccupancyGrid& map) const;
    void printPdf(double trueDist, double granularity, std::ostream& out) const;

private:
    
    ///////// TODO: Add any private members for your SensorModel ///////////////////
    double _rangeStd;
    double _rangeMax;
    // parameter for tuning how we hit obstacles
    double _hitObstacle;
    // parameter for tuning how often we detect nothing
    double _detectNothing;
    // parameter for tuning how often random measurements show up
    double _detectRandom;
    // parameter for odds above which we deem is a wall
    CellOdds _wallOdds;
};

#endif // SLAM_SENSOR_MODEL_HPP
