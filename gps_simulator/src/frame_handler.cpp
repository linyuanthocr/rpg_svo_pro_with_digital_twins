

#include "frame_handler.hpp"
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>



void ENUtoECEF(double east, double north, double up,
               double refLat, double refLon, double refAlt,
               double &ecefX, double &ecefY, double &ecefZ) {

    const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();
    //local frame centered at the origin 
    GeographicLib::LocalCartesian enu(refLat, refLon, refAlt, earth);
    double lat, lon, alt; 

    // from enu to lon lat alt of the point we want to convert in ecef
    ENUToLLA(refLat, refLon, refAlt, 
                        east, north, up,
                        lat, lon, alt) ;

    // from lon lat alt ecef cordinates
    earth.Forward(lat, lon, alt, ecefX, ecefY, ecefZ);

}

void ENUToLLA(double refLat, double refLon, double refAlt, 
                     double enuE, double enuN, double enuU,
                     double& lat, double& lon, double& alt) {

    // Initialize the LocalCartesian model with the reference point
    GeographicLib::LocalCartesian proj(refLat, refLon, refAlt);

    // Convert ENU to geographic coordinates
    proj.Reverse(enuE, enuN, enuU, lat, lon, alt);
}



void LLAToECEF(double lat, double lon, double alt,
                      double& x, double& y, double& z) {
    // Use the WGS84 ellipsoid
    const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();
    // Convert geographic to ECEF coordinates
    earth.Forward(lat, lon, alt, x, y, z);
}

void ECEFToLLA(double x, double y, double z,
                      double& lat, double& lon, double& alt) {
    // Use the WGS84 ellipsoid
    const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();
    // Convert ECEF to geographic coordinates
    earth.Reverse(x, y, z, lat, lon, alt);
}

void LLAtoENU(double lat, double lon, double alt, double refLat, double refLon, double refAlt, double &east, double &north, double &up) {
    // Initialize the LocalCartesian model with the reference point
    GeographicLib::LocalCartesian proj(refLat, refLon, refAlt);
    // Convert ECEF to ENU coordinates
    proj.Forward(lat, lon, alt, east, north, up);
}
