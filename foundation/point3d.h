// point3d.cpp

#pragma once

#include "../pch.h"
#include <boost/geometry/geometries/register/point.hpp>

namespace spatial {
    
    class CartesianPoint3D {
    public:
        CartesianPoint3D() : x(0), y(0), z(0) {}

        const double u() const { return y; }
        const double v() const { return x; }
        const double w() const { return z; }

        const double L() const { return x; }
        const double M() const { return y; }
        const double N() const { return z; }

        void u( const double u ) { y = u; }
        void v( const double v ) { x = v; }
        void w( const double w ) { z = w; }

        void L( const double L ) { x = L; }
        void M( const double M ) { y = M; }
        void N( const double N ) { z = N; }

        double x, y, z; 
    };

    class GeoPointRad3D {
    public:
        GeoPointRad3D() : lat(0.0), lon(0.0), alt(0.0) {}
        double lat;
        double lon; 
        double alt;
    };

    class GeoPointDeg3D : public GeoPointRad3D {};
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

