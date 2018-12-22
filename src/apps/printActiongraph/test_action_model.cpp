#include <slam/particle_filter.hpp>
#include <slam/action_model.hpp>

#include <iostream>
using namespace std;

int main(int argc, char** argv) {
     if (argc != 7) {
         std::cerr << "Specify init x,y,theta and moveto x,y,theta (rad)\n";
         return -1;
     }

    // auto trueDist = std::stod(argv[1]);
    // auto granularity = std::stod(argv[2]);

    // move simply
    pose_xyt_t odometry;
    odometry.x = std::stod(argv[4]);
    odometry.y = std::stod(argv[5]);
    odometry.theta = std::stod(argv[6]);
    const int NumOfParticles = 1000;


    particle_t init_particle;
    init_particle.pose.x = std::stod(argv[1]);
    init_particle.pose.y = std::stod(argv[2]);
    init_particle.pose.theta = std::stod(argv[3]);
    printf("%f,%f,%f\n", init_particle.pose.x, init_particle.pose.y, init_particle.pose.theta);

    ActionModel actionModel(init_particle.pose.x, init_particle.pose.y, init_particle.pose.theta);
    // initialize action model at initial point
    // actionModel.updateAction(init_particle.pose);
    if (actionModel.updateAction(odometry)) {
        std::vector<particle_t> particle_list;
        for (int i=0;i<NumOfParticles;i++) {
            particle_list.push_back(actionModel.applyAction(init_particle));
            printf("%f,%f,%f\n", particle_list[i].pose.x,particle_list[i].pose.y,particle_list[i].pose.theta);
        }
    }
}

