#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <random>
#include <iomanip>

ParticleFilter::ParticleFilter(int numParticles)
        : actionModel_(0, 0, 0)
        , kNumParticles_(numParticles) {
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t &pose) {
    std::default_random_engine generator;

    std::cout << "Initializating particles\n";
    // arbitrarily select standard deviation
    std::normal_distribution<decltype(pose_xyt_t::x)> dx(pose.x, 0.1);
    std::normal_distribution<decltype(pose_xyt_t::y)> dy(pose.y, 0.1);
    std::normal_distribution<decltype(pose_xyt_t::theta)> dtheta(pose.theta, 0.5);

    for (auto i = 0; i < kNumParticles_; ++i) {
        posterior_[i].pose.x = pose.x; //dx(generator);
        posterior_[i].pose.y = pose.y; // dy(generator);
        posterior_[i].pose.theta = pose.theta; //dtheta(generator);
        posterior_[i].parent_pose = posterior_[i].pose;
        posterior_[i].weight = 1;
    }

    //        std::sort(posterior_.begin(), posterior_.end(),
    //              [](const particle_t &a, const particle_t &b) { return a.weight > b.weight; });
    //    std::cout << "after update\n";
    //    for (const auto &p: posterior_) {
    //        std::cout << std::setw(15) << p.pose.x << std::setw(15) << p.pose.y << std::setw(15) << p.pose.theta
    //                  << std::setw(15) << p.weight << std::endl;
    //    }
}

pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t &odometry,
                                        const lidar_t &laser,
                                        const OccupancyGrid &map) {
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if (hasRobotMoved) {
        posterior_ = resamplePosteriorDistribution();

        // prior right after resampling
        computeProposalDistribution(posterior_);
        // prior becomes proposal after applying action model
        computeNormalizedPosterior(posterior_, laser, map);
        // proposal becomes posterior after applying sensor model and normalizing
        posteriorPose_ = estimatePosteriorPose(posterior_);

//        // print after sorting by weight
    //    std::sort(posterior_.begin(), posterior_.end(),
    //              [](const particle_t &a, const particle_t &b) { return a.weight > b.weight; });
    //    std::cout << "after update\n";
    //    for (const auto &p: posterior_) {
    //        std::cout << std::setw(15) << p.pose.x << std::setw(15) << p.pose.y << std::setw(15) << p.pose.theta
    //                  << std::setw(15) << p.weight << std::endl;
    //    }
//        exit(0);
    }

    posteriorPose_.utime = odometry.utime;

    return posteriorPose_;
}


pose_xyt_t ParticleFilter::poseEstimate(void) const {
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const {
    particles_t particles;
    particles.num_particles = static_cast<decltype(particles_t::num_particles)>(posterior_.size());
    particles.particles = posterior_;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void) {
    // assume our posterior covers the entire region we need to sample so we don't have to generate new particles
    // use CDF based approach
    // if random value lands between cdf(i-1) and cdf(i) then choose particle i
    std::vector<double> cdf;
    cdf.reserve(kNumParticles_);
    double currentCdf = 0;
    for (const auto &particle : posterior_) {
        currentCdf += particle.weight;
        cdf.push_back(currentCdf);
    }

    std::vector<particle_t> prior;
    prior.reserve(kNumParticles_);

    // randomly sample N numbers uniformly from 0 to currentCdf then find the corresponding particle
    // note that CDF is already sorted (non decreasing) so can use binary search
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, currentCdf);
    const auto b = cdf.begin();
    const auto e = cdf.end();
    for (int i = 0; i < kNumParticles_; ++i) {
        const auto sampledParticleIndex = std::lower_bound(b, e, dis(gen)) - b;
        prior.push_back(posterior_[sampledParticleIndex]);
    }

    return prior;
}


void ParticleFilter::computeProposalDistribution(std::vector<particle_t> &prior) {
    for (int i = 0; i < kNumParticles_; ++i) {
        prior[i] = actionModel_.applyAction(prior[i]);
    }
}


void ParticleFilter::computeNormalizedPosterior(std::vector<particle_t> &proposal,
                                                const lidar_t &laser,
                                                const OccupancyGrid &map) {
    ///////////       particles in the proposal distribution
    double totalWeight = 0;
    for (auto &particle : proposal) {
        particle.weight *= sensorModel_.likelihood(particle, laser, map);
        totalWeight += particle.weight;
    }

    // normalize weights (avoid saturating double)
    const auto normalizationTerm = 1 / totalWeight;
//    std::cout << "Total likelihood " << totalWeight << " normalization term " << normalizationTerm << std::endl;
    for (auto &particle : proposal) {
        particle.weight *= normalizationTerm;
//        std::cout << particle.pose.x << ' ' << particle.pose.y << ' ' << particle.pose.theta << ' ' << particle.weight << std::endl;
    }
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t> &posterior) {
    // can choose either max weight particle or center of mass (we'll use center of mass)
    auto pose = pose_xyt_t();
    pose.x = 0;
    pose.y = 0;
    pose.theta = 0;
    double totalWeight = 0;
    for (int i = 0; i < kNumParticles_; ++i) {
        const auto &particle = posterior[i];
        pose.x += particle.pose.x * particle.weight;
        pose.y += particle.pose.y * particle.weight;
        pose.theta += particle.pose.theta * particle.weight;
        totalWeight += particle.weight;
    }
    pose.x /= totalWeight;
    pose.y /= totalWeight;
    pose.theta /= totalWeight;
    pose.theta = wrap_to_pi(pose.theta);
    return pose;
}
