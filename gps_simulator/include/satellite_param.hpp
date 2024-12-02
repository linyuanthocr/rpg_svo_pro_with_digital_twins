#ifndef SATELLITEPARAM_H
#define SATELLITEPARAM_H

#include <cmath>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

class SatelliteParam {
public:
    double muE;  // Earth's mass
    double RE;   // Earth's radius

    double sa;           // semi-major axis 
    double mu_anomalie;  // mean anomaly
    double arg;          // argument of periapsis
    double orbinc;       // orbital inclination
    double RAANangle;    // right ascension of the ascending node
    double eccen;        // eccentricity

    int satellite_number_;

  
    SatelliteParam(const std::string& filename, int satellite_number) ;

    SatelliteParam() ;

    ~SatelliteParam() {}
};

#endif // SATELLITEPARAM_H
