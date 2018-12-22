// Make use of the CMake build system or compile manually, e.g. with:
// g++ -std=c++11 example.cc -lsweep


#include <signal.h>
#include <sys/time.h>

#include <cstdlib>
#include <iostream>
#include <cmath>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/lidar_t.hpp>

#include <common/sweep.hpp>
#include <common/lcm_config.h> 
#include <common/timestamp.h>

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int main(int argc, char* argv[]) try {
  const char * opt_com_path = NULL;
  int32_t rotationHZ = 5;
  int32_t sampleHZ = 1000;
  ctrl_c_pressed = false;

  lcm::LCM lcmConnection(MULTICAST_URL);

  if(!lcmConnection.good()){
      return 1;
  }

  //get rotation rate if specified...
  if (argc > 1) rotationHZ = atoi(argv[1]);
  //get sample rte if specified...
  if (argc > 2) sampleHZ = atoi(argv[2]);
  //get serial port name if specified...
  if (argc > 3) opt_com_path = argv[3]; 

    if (!opt_com_path) {
#ifdef _WIN32
        // use default com port
        opt_com_path = "\\\\.\\com3";
#else
        opt_com_path = "/dev/ttyUSB0";
#endif
    }

  std::cout << "Constructing sweep device..." << std::endl;
  sweep::sweep device{opt_com_path};

  std::cout << "setting rotation rate to: " << rotationHZ << " HZ, and sample rate to: " << sampleHZ << " HZ\n";

  if(rotationHZ != device.get_motor_speed())  device.set_motor_speed(rotationHZ);
  if(sampleHZ != device.get_sample_rate()){
	 std::cout << "setting sample rate to: " << sampleHZ << " HZ.\n";
	 device.set_sample_rate(sampleHZ);
  }
  std::cout << "Motor Speed Setting: " << device.get_motor_speed() << " Hz" << std::endl;
  std::cout << "Sample Rate Setting: " << device.get_sample_rate() << " Hz" << std::endl;

  assert(device.get_motor_speed() != 0);

  std::cout << "Beginning data acquisition as soon as motor speed stabilizes..." << std::endl;

  signal(SIGINT, ctrlc);
  signal(SIGTERM, ctrlc);

  device.start_scanning();

  int64_t end_time = utime_now();
  int64_t delta_time;
  int64_t start_time;
  int pos = 0;

  while(1) {

    start_time = end_time;
    //blocks until scan is complete
    const sweep::scan scan = device.get_scan();
    end_time = utime_now();
    delta_time = end_time - start_time;
    
    lidar_t newLidar;

    newLidar.utime = start_time;
    newLidar.num_ranges = scan.samples.size();
    newLidar.ranges.resize(newLidar.num_ranges);
    newLidar.thetas.resize(newLidar.num_ranges);
    newLidar.intensities.resize(newLidar.num_ranges);
    newLidar.times.resize(newLidar.num_ranges);

    pos = 0;
    for (const sweep::sample& sample : scan.samples) {
      newLidar.ranges[pos] = sample.distance / 100.0f;
      newLidar.thetas[pos] = (2 * M_PI) - (sample.angle * M_PI / 180000.0f);
      newLidar.intensities[pos] = sample.signal_strength;
      newLidar.times[pos] = start_time + pos*delta_time;

      ++pos;
    }

    lcmConnection.publish("LIDAR", &newLidar);

    if (ctrl_c_pressed){ 
      break;
    }




  }

  device.stop_scanning();
} catch (const sweep::device_error& e) {
  std::cerr << "Error: " << e.what() << std::endl;
}
