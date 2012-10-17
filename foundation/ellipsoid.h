// ellipsoid.h

/**
 *    Copyright (C) 2012 Thermopylae Sciences + Technology
 * 
 *    This program is free software: you can redistribute it and/or  modify
 *    it under the terms of the GNU Affero General Public License, version 3,
 *    as published by the Free Software Foundation.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Affero General Public License for more details.
 *
 *    You should have received a copy of the GNU Affero General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "../pch.h"
#include "point3d.h"

namespace spatial {

    class Ellipsoid {
    public:
        /**
         * Constructor defaults to code 7030, 
         * which is the EPG code for the WGS84 ellipsoid
         */
        Ellipsoid( const unsigned short code = 7030 ) : _code(code) { initialize(); }

        // - conversions 
        //      global assumptions: 
        //          *this* object is not modified, new objects are returned
        //          angles are in radians except 
        
        // converts from earth-centered earth-fixed cartesian to lat,lon,alt spherical
        const Point3D<GeoRad>& ecfToLatLonAlt( const Point3D<Cartesian>& ecfPt ) const;
        const Point3D<GeoRad>& enuToLatLonAlt( const Point3D<Cartesian>& enuPt, 
                                               const Point3D<GeoRad>& refPt ) const;
        const Point3D<GeoRad>& utmToLatLonAlt( const Point3D<Cartesian>& utmPt ) const;
        const Point3D<GeoRad>& mgrsToLatLonAlt( const string& mgrsPt ) const;

        const Point3D<Cartesian>& latLonAltToECF( const Point3D<GeoRad>& llaPt ) const;
        const Point3D<Cartesian>& enuToECF( const Point3D<Cartesian>& enuPt, 
                                            const Point3D<GeoRad>& refPt ) const;
        const Point3D<Cartesian>& utmToECF( const Point3D<Cartesian>& utmPt ) const;
        const Point3D<Cartesian>& mgrsToECF( const string& mgrsPt ) const;

        const Point3D<Cartesian>& latLonAltToUTM( const Point3D<GeoRad>& llaPt ) const;
        const Point3D<Cartesian>& enuToUTM( const Point3D<Cartesian>& enuPt, 
                                            const Point3D<GeoRad>& refPt ) const;
        const Point3D<Cartesian>& ecfToUTM( const Point3D<Cartesian>& ecfPt ) const;
        const Point3D<Cartesian>& mgrsToUTM( const string& mgrsPt ) const;

        const string& latLonAltToMGRS( const Point3D<GeoRad>& llaPt ) const;
        const string& ecfToMGRS( const Point3D<Cartesian>& ecfPt ) const;
        const string& enuToMGRS( const Point3D<Cartesian>& enuPt, 
                                 const Point3D<GeoRad>& refPt ) const;
        const string& utmToMGRS( const Point3D<Cartesian>& utmPt ) const;

        // convenience methods to handle degrees
        void ecfToLatLonAltDeg( const Point3D<Cartesian>& ecfPt,
                                Point3D<GeoDeg> &llaDeg) const {
            Point3D<GeoRad> rad = ecfToLatLonAlt( ecfPt );
            transform(rad, llaDeg);
        }
        void enuToLatLonAltDeg( const Point3D<Cartesian>& enuPt, const Point3D<GeoRad>& refPt,
                                Point3D<GeoDeg> &llaDeg) const {
            Point3D<GeoRad> rad = enuToLatLonAlt( enuPt, refPt );
            transform(rad, llaDeg);
        }
        void utmToLatLonAltDeg( const Point3D<Cartesian>& utmPt,
                                Point3D<GeoDeg> &llaDeg) const {
            Point3D<GeoRad> rad = utmToLatLonAlt( utmPt );
            transform(rad, llaDeg);
        }
        void mgrsToLatLonAltDeg( const string& mgrsPt,
                                 Point3D<GeoDeg>& llaDeg) const {
            Point3D<GeoRad> llaRad = mgrsToLatLonAlt( mgrsPt );
            transform(llaRad, llaDeg);
        }
        void latLonAltDegToECF( const Point3D<GeoDeg>& llaDeg, 
                                Point3D<Cartesian> &ecef) const {
            Point3D<GeoRad> llaRad;
            transform(llaDeg, llaRad);
            ecef = latLonAltToECF( llaRad );
        }
        void latLonAltDegToUTM( const Point3D<GeoDeg>& llaDeg,
                                Point3D<Cartesian>& utm) const {
            Point3D<GeoRad> llaRad;
            transform(llaDeg, llaRad);
            utm = latLonAltToUTM( llaRad );
        }
        void latLonAltDegToMGRS( const Point3D<GeoDeg>& llaDeg, string& mgrs ) const {
            Point3D<GeoRad> llaRad;
            transform(llaDeg, llaRad);
            mgrs = latLonAltToMGRS( llaRad );
        }
        // - end conversions

        // getters
        unsigned short code() { return _code; }
        double semiMajorAxis() { return _semiMajorAxis; }
        double semiMinorAxis() { return _semiMinorAxis; }
        double flattening() { return _flattening; }
        double eccentricity() { return _eccentricity; }

    protected:

        // initializes this object using the code value 
        // specified by the constructor
        void initialize();

        // convergence criteria for ecf to lla conversion
        // defined in 'The Manual of Photogrammetry'
        static const double _ITERATION_THRESHOLD = 4.8481368110953599e-08;

        // print this ellipsoid object
        // @TODO: Replace this operation with a string buffer
        friend ostream& operator << ( ostream& os, const Ellipsoid& ellipsoid );

    private:

        // private attributes
        unsigned short _code;
        double _semiMajorAxis;
        double _semiMinorAxis;
        double _flattening;
        double _eccentricity;
    };


}
