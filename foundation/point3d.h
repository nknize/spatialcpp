// point3d.cpp

#pragma once

#include "../pch.h"
#include <boost/geometry/geometries/point.hpp>

namespace spatial {

    template <typename T>
    class Point3D : public model::point<double, 3, T> {
    public:
        Point3D() {}


    };

BOOST_GEOMETRY_REGISTER_POINT_3D(Point3D, double, typename T);
}
