#include <slam/sensor_model.hpp>
#include <iostream>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Specify true distance (m) and granularity to print (m)\n";
        return -1;
    }

    auto trueDist = std::stod(argv[1]);
    auto granularity = std::stod(argv[2]);
    const auto sensorModel = SensorModel();
    sensorModel.printPdf(trueDist, granularity, std::cout);
}