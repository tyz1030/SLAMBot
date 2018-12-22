=== Directories ===

= bin/
    - where all built binaries are located
    - you'll be using this directory a lot
    
= data/
    - where data needed to run parts of the assignment are located
    - log files and target files for SLAM and exploration are here
    
= lcmtypes/
    - where the .lcm type definitions are located
    - the generated types are stored in src/lcmtypes
    
= lib/
    - where static libraries are saved during the build process
    - you should never need to manually do anything in this directory
    
= src/
    - where all source code for botlab is located
    - the subdirectories will have a further description of their contents
    

=== Files ===

= Makefile
    - the root Makefile that launches the recursive build of the botlab code
    - you shouldn't need to edit this file
    
= log_mbot_sensors.sh
    - a script to log the sensor data needed for SLAM so you can easily create your own log files 
      for testing

= setenv.sh
    - a script for setting environment variables needed for running Vx applications
    - run this script before running botgui in a terminal on your laptop
    - run via `. setenv.sh` -- note the space

## What to do in each section
2.1
    - need optitrack ground truth
        - how to get optitrack ground truth? Is mobilebot broadcasting optitrack?
        - what is OPTITRACK_CHANNEL?
3.1
    - what is the difference between sweep lidar and rplidar?
    - modify src/mapping
    - run `slam --mapping-only`
3.2
    - action model
    - sensor model
