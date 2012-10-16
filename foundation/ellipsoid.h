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

namespace spatial {

    class Ellipsoid {
    public:
        /**
         * Constructor defaults to code 7030, 
         * which is the EPG code for the WGS84 ellipsoid
         */
        Ellipsoid( const unsigned short code = 7030 ) : _code(code) { initialize() };

        // - conversions 
        //      global assumptions: 
        //          *this* object is not modified, new objects are returned
        //          angles are in radians except 
        
        // converts from earth-centered earth-fixed cartesian to lat,lon,alt spherical
        const Point3D& ecfToLatLonAlt( const Point3D& ecfPt ) const;
        const Point3D& enuToLatLonAlt( const Point3D& enuPt, const Point3D& refPt ) const;
        const Point3D& utmToLatLonAlt( const Point3D& utmPt ) const;
        const Point3D& mgrsToLatLonAlt( const string& mgrsPt ) const;

        const Point3D& latLonAltToECF( const Point3D& llaPt ) const;
        const Point3D& enuToECF( const Point3D& enuPt, const Point3D& refPt ) const;
        const Point3D& utmToECF( const Point3D& utmPt ) const;
        const Point3D& mgrsToECF( const string& mgrsPt ) const;

        const Point3D& latLonAltToUTM( const Point3D& llaPt ) const;
        const Point3D& enuToUTM( const Point3D& enuPt, const Point3D& refPt ) const;
        const Point3D& ecfToUTM( const Point3D& ecfPt ) const;
        const Point3D& mgrsToUTM( const string& mgrsPt ) const;

        const string& latLonAltToMGRS( const Point3D& llaPt ) const;
        const string& ecfToMGRS( const Point3D& ecfPt ) const;
        const string& enuToMGRS( const Point3D& enuPt, const Point3D& refPt ) const;
        const string& utmToMGRS( const Point3D& utmPt ) const;

        // convenience methods to handle degrees
        const Point3D& ecfToLatLonAltDeg( const Point3D& ecfPt ) const {
            Point3D llaPt = ecfToLatLonAlt( ecfPt );
            return llaPt.toDegrees();
        }
        const Point3D& enuToLatLonAltDeg( const Point3D& enuPt, const Point3D& refPt ) const {
            Point3D llaPt = enuToLatLonAlt( enuPt, refPt );
            return llaPt.toDegrees();
        }
        const Point3D& utmToLatLonAltDeg( const Point3D& utmPt ) const {
            Point3D llaPt = utmToLatLonAlt( utmPt );
            return llaPt.toDegrees();
        }
        const Point3D& mgrsToLatLonAltDeg( const string& mgrsPt ) const {
            Point3D llaPt = mgrsToLatLonAltDeg( mgrsPt );
            return llaPt.toDegrees();
        }
        const Point3D& latLonAltDegToECF( const Point3D& llaPt ) const {
            return latLonAltToECF( llaPt.toDegrees() );
        }
        const Point3D& latLonAltDegToUTM( const Point3D& llaPt ) const {
            return latLonAltToUTM( llaPt.toDegrees() );
        }
        const string& latLonAltDegToMGRS( const Point3D& llaPt ) const {
            return latLonAltToMGRS( llaPt.toDegrees );
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

        unsigned short _code;
        double _semiMajorAxis;
        double _semiMinorAxis;
        double _flattening;
        double _eccentricity;
    };

}
