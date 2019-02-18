# 18d: Balancing with COM Error
This project attempts to balance the robot with an incorrect center of mass estimation using ADRC.

### Dependencies
- DART (at least version 6) [Dart Homepage](https://dartsim.github.io)
- LAPACK [LAPACK Homepage](www.netlib.org/lapack)
  * On Ubuntu systems: `sudo apt install liblapack-dev`

### Build and Run
1: Enter the repository

2: Build the project

    mkdir build
    cd build
    cmake ..
    make

3: Run the project

    ./balancingWithCoMError
