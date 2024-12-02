#include "satellite_param.hpp"
#include <iostream>


SatelliteParam::SatelliteParam(const std::string& filename, int satellite_number) : satellite_number_(satellite_number) {

    YAML::Node config = YAML::LoadFile(filename);

    if (!config["RE"] || !config["muE"]) {
        throw std::runtime_error("Required parameters RE or muE not found in YAML file.");
    }

    RE = config["RE"].as<double>();
    muE = config["muE"].as<double>();

    auto load_vector = [&](const YAML::Node& node, std::vector<double>& vec) {
        if (!node) {
            throw std::runtime_error("Required parameter missing in YAML file.");
        }
        for (const auto& val : node) {
            vec.push_back(val.as<double>());
        }
    };

    if (satellite_number < 0 || satellite_number >= config["sqrt_sa"].size()) {
        throw std::out_of_range("Satellite number is out of range.");
    }

    // Load each parameter as a single value
    sa = std::pow(config["sqrt_sa"][satellite_number].as<double>(), 2);
    mu_anomalie = config["mu_anomalie"][satellite_number].as<double>();
    arg = config["arg"][satellite_number].as<double>();
    orbinc = config["orbinc"][satellite_number].as<double>();
    RAANangle = config["RAANangle"][satellite_number].as<double>();
    eccen = config["eccen"][satellite_number].as<double>();
    
}

SatelliteParam::SatelliteParam() {
        
    sa = std::pow(5153.653320, 2);
    mu_anomalie = -0.1208933778e+001;
    arg = 0.563527841;
    orbinc = 0.9651879023;
    RAANangle = 0.1489908052e+001;
    eccen = 0.5968570709e-002;

    RE = 6378000;
    muE = 3.986004418e+14;

}

