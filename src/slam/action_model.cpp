#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <math.h>
#include <random>
#include <fstream>


ActionModel::ActionModel(float x, float y, float theta) {
    // std::default_random_engine generator;
    std::ifstream f{"action_model.cfg"};
    f >> alpha1 >> alpha2 >> alpha3 >> alpha4;

    prev_odometry.x = x;
    prev_odometry.y = y;
    prev_odometry.theta = theta;

    std::random_device rd{};
    gen = std::mt19937{rd()};
}


// This is the algorithm shown in Page 134-Table 5.5
// Need to update the true_pose and true_pose_dot in particle filter
bool ActionModel::updateAction(const pose_xyt_t &odometry) {
    // extract estimated motion parameters (rot, trans, rot) from odometry
    x_bar_new = odometry.x;
    y_bar_new = odometry.y;
    theta_bar_new = odometry.theta;

    x_bar = prev_odometry.x;
    y_bar = prev_odometry.y;
    theta_bar = prev_odometry.theta;
    std::cout << "prev odo " << prev_odometry.x << " " << prev_odometry.y << " " << prev_odometry.theta << std::endl;

    if ((y_bar_new == y_bar) && (x_bar_new = x_bar)){
            delta_rot1 = 0;
    }else{
            delta_rot1 = atan2(y_bar_new - y_bar, x_bar_new - x_bar) - theta_bar;

    };
    delta_trans = sqrt(pow(x_bar_new - x_bar, 2) + pow(y_bar_new - y_bar, 2));
    delta_rot2 = angle_diff(angle_diff(theta_bar_new, theta_bar), delta_rot1);

    if (fabs(delta_rot1) < 1e-5 && fabs(delta_trans) < 1e-5) {
        return false;
    }

    const auto b1 = alpha1 * pow(delta_rot1, 2) + alpha2 * pow(delta_trans, 2);
    const auto b2 = alpha3 * pow(delta_trans, 2) + alpha4 * pow(delta_rot1, 2) + alpha4 * pow(delta_rot2, 2);
    const auto b3 = alpha1 * pow(delta_rot2, 2) + alpha2 * pow(delta_trans, 2);

    p1 = std::normal_distribution<float>(0.0, sqrt(b1));
    p2 = std::normal_distribution<float>(0.0, sqrt(b2));
    p3 = std::normal_distribution<float>(0.0, sqrt(b3));

    prev_odometry = odometry;
    return true;
}

// This is the algorithm shown in Page 136-Table 5.6
particle_t ActionModel::applyAction(const particle_t &sample) {
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    const auto delta_rot1_hat = angle_diff(delta_rot1, p1(gen));
    const auto delta_trans_hat = delta_trans - p2(gen);
    const auto delta_rot2_hat = angle_diff(delta_rot2, p3(gen));
//    std::cout << "hat delta trans " << delta_trans_hat << " " << delta_rot1_hat << " " << delta_rot2_hat << std::endl;


    auto new_sample = particle_t();
    new_sample.parent_pose = sample.pose;
    new_sample.pose.x = sample.pose.x + delta_trans_hat * cos(sample.pose.theta + delta_rot1_hat);
    new_sample.pose.y = sample.pose.y + delta_trans_hat * sin(sample.pose.theta + delta_rot1_hat);
    new_sample.pose.theta = wrap_to_pi(sample.pose.theta + delta_rot1_hat + delta_rot2_hat);
    new_sample.weight = sample.weight;

    // decide weight based on Page 252-Table 8.2

    return new_sample;
}

