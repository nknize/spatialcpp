// point3d.cpp

#pragma once

#include "../pch.h"
#include <boost/geometry/geometries/register/point.hpp>

namespace spatial {
    
    class CartesianPoint3D {
      friend class Ellipsoid;
    public:
        CartesianPoint3D() : x(0), y(0), z(0) {}
        CartesianPoint3D(const double x_, const double y_, const double z_) : x(x_), y(y_), z(z_) {}

        // move constructor
        CartesianPoint3D(CartesianPoint3D &&other)
          : x {other.x},
            y {other.y},
            z {other.z} {
          cout << "calling move ctor in cart point" << endl;
          cout << this->x << "    " << this->y << "   " << this->z << endl;
          other.x = 0;
          other.y = 0;
          other.z = 0;
        }

        // move operator
        CartesianPoint3D& operator=(CartesianPoint3D&& other) {
          if(this != &other) {
              cout << "calling move assignment op in cart point" << endl;
            x = other.x;
            y = other.y;
            z = other.z;

            other.x = 0;
            other.y = 0;
            other.z = 0;
          }
          return *this;
        }

        string toString() const {
          ostringstream oss;
          oss << "(" << x << ", " << y << ", " << z << ")";
          return oss.str();
        }

        inline const double u() const { return y; }
        inline const double v() const { return x; }
        inline const double w() const { return z; }

        inline const double L() const { return x; }
        inline const double M() const { return y; }
        inline const double N() const { return z; }

        inline void u( const double u ) { y = u; }
        inline void v( const double v ) { x = v; }
        inline void w( const double w ) { z = w; }

        inline void L( const double L ) { x = L; }
        inline void M( const double M ) { y = M; }
        inline void N( const double N ) { z = N; }

        double x, y, z;
    };

    class GeoPointRad3D {
     public:
        GeoPointRad3D() : lat(0.0), lon(0.0), alt(0.0) {}
        GeoPointRad3D(double lat_, double lon_, double alt_) : lat(lat_), lon(lon_), alt(alt_) {}
    
        // move constructor
        GeoPointRad3D(GeoPointRad3D &&other)
          : lat {other.lat},
            lon {other.lon},
            alt {other.alt} {
          cout << "calling move ctor in geo point rad" << endl;
          cout << this->lat << "    " << this->lon << "   " << this->alt << endl;
          other.lat = 0;
          other.lon = 0;
          other.alt = 0;
        }

        // move operator
        GeoPointRad3D& operator=(GeoPointRad3D&& other) {
          if(this != &other) {
              cout << "calling move assignment op in cart point" << endl;
            lat = other.alt;
            lon = other.lon;
            alt = other.alt;

            other.lat = 0;
            other.lon = 0;
            other.alt = 0;
          }
          return *this;
        }

        // airframe terminology
        /** @TODO Double check yaw stored as angle not scalar */
        const double roll() const { return lon; }
        const double pitch() const { return lat; }
        const double yaw() const { return alt; }
        
        void roll( const double roll ) { lon = roll; }
        void pitch( const double pitch ) { lat = pitch; }
        void yaw( const double yaw ) { alt = yaw; }

        string toString() const {
          ostringstream oss;
          oss << "(" << lat << ", " << lon << ", " << alt << ")";
          return oss.str();
        }

        double lat, lon, alt;
    };

    class GeoPointDeg3D : public GeoPointRad3D {
     public:
      GeoPointDeg3D() : GeoPointRad3D() {}
      GeoPointDeg3D(double lat_, double lon_, double alt_)
        : GeoPointRad3D(lat_, lon_, alt_) {}
      GeoPointDeg3D(GeoPointDeg3D &&other)
        : GeoPointRad3D{std::move(other)} {}
      GeoPointDeg3D& operator=(GeoPointDeg3D&& other) {
        if(this != &other) {
            cout << "calling move assignment op in geoptdeg3d point" << endl;
          *this = std::move(other);
        }
        return *this;
      }
    };
    class CartesianPoint2D : public CartesianPoint3D {};
    class GeoPointRad2D : public GeoPointRad3D {};
    class GeoPointDeg2D : public GeoPointDeg3D {};

}

BOOST_GEOMETRY_REGISTER_POINT_3D(spatial::CartesianPoint3D, double, spatial::Cartesian, x, y, z);
BOOST_GEOMETRY_REGISTER_POINT_3D(spatial::GeoPointRad3D, double, spatial::GeoRad, lon, lat, alt);
BOOST_GEOMETRY_REGISTER_POINT_3D(spatial::GeoPointDeg3D, double, spatial::GeoDeg, lon, lat, alt);

BOOST_GEOMETRY_REGISTER_POINT_2D(spatial::CartesianPoint2D, double, spatial::Cartesian, x, y);
BOOST_GEOMETRY_REGISTER_POINT_2D(spatial::GeoPointRad2D, double, spatial::GeoRad, lon, lat);
BOOST_GEOMETRY_REGISTER_POINT_2D(spatial::GeoPointDeg2D, double, spatial::GeoDeg, lon, lat);

