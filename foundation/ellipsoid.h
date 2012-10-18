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
        void ecfToLatLonAlt( const CartesianPoint3D& ecfPt, GeoPointRad3D &geoPt) const;
        void enuToLatLonAlt( const CartesianPoint3D& enuPt, const GeoPointRad3D& refPt,
                             GeoPointRad3D &geoPt) const;
        void utmToLatLonAlt( const CartesianPoint3D& utmPt, GeoPointRad3D &geoPt ) const;
        void mgrsToLatLonAlt( const string& mgrsPt, GeoPointRad3D &geoPt ) const;

        void latLonAltToECF( const GeoPointRad3D& llaPt, CartesianPoint3D &ecfPt ) const;
        void enuToECF( const CartesianPoint3D& enuPt, const GeoPointRad3D& refPt, 
                       CartesianPoint3D &ecfPt ) const;
        void utmToECF( const CartesianPoint3D& utmPt, CartesianPoint3D &ecfPt ) const;
        void mgrsToECF( const string& mgrsPt, CartesianPoint3D &ecfPt ) const;

        void latLonAltToUTM( const GeoPointRad3D& llaPt, CartesianPoint3D &utmPt ) const;
        void enuToUTM( const CartesianPoint3D& enuPt, const GeoPointRad3D& refPt, 
                       CartesianPoint3D &utmPt ) const;
        void ecfToUTM( const CartesianPoint3D& ecfPt, CartesianPoint3D& utmPt ) const;
        void mgrsToUTM( const string& mgrsPt, CartesianPoint3D &utmPt ) const;

        void latLonAltToMGRS( const GeoPointRad3D& llaPt, string &mgrsPt ) const;
        void ecfToMGRS( const CartesianPoint3D& ecfPt, string &mgrsPt ) const;
        void enuToMGRS( const CartesianPoint3D& enuPt, const GeoPointRad3D& refPt, 
                        string &mgrsPt) const;
        void utmToMGRS( const CartesianPoint3D& utmPt, string &mgrsPt ) const;

        // convenience methods to handle degrees
        void ecfToLatLonAltDeg( const CartesianPoint3D& ecfPt,
                                GeoPointDeg3D &llaDeg) const {
            GeoPointRad3D rad; 
            ecfToLatLonAlt( ecfPt, rad );
            transform(rad, llaDeg);
        }
        void enuToLatLonAltDeg( const CartesianPoint3D& enuPt, const GeoPointRad3D& refPt,
                                GeoPointDeg3D &llaDeg) const {
            GeoPointRad3D rad;
            enuToLatLonAlt( enuPt, refPt, rad );
            transform(rad, llaDeg);
        }
        void utmToLatLonAltDeg( const CartesianPoint3D& utmPt,
                                GeoPointDeg3D &llaDeg) const {
            GeoPointRad3D rad;
            utmToLatLonAlt( utmPt, rad );
            transform(rad, llaDeg);
        }
        void mgrsToLatLonAltDeg( const string& mgrsPt,
                                 GeoPointDeg3D& llaDeg) const {
            GeoPointRad3D llaRad;
            mgrsToLatLonAlt( mgrsPt, llaRad );
            transform(llaRad, llaDeg);
        }
        void latLonAltDegToECF( const GeoPointDeg3D& llaDeg, 
                                CartesianPoint3D &ecef) const {
            GeoPointRad3D llaRad;
            transform(llaDeg, llaRad);
            latLonAltToECF( llaRad, ecef );
        }
        void latLonAltDegToUTM( const GeoPointDeg3D& llaDeg,
                                CartesianPoint3D& utm) const {
            GeoPointRad3D llaRad;
            transform(llaDeg, llaRad);
            latLonAltToUTM( llaRad, utm );
        }
        void latLonAltDegToMGRS( const GeoPointDeg3D& llaDeg, string& mgrs ) const {
            GeoPointRad3D llaRad;
            transform(llaDeg, llaRad);
            latLonAltToMGRS( llaRad, mgrs );
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

        // ref: surveying.wb.psu.edu/sur351/georef/ellip3.htm (formula 7)
        const double primeVertRadiusOfCurvature( const double lat ) const {
            const double denom = _eccentricity * sin( lat );
            return (_semiMajorAxis / sqrt( 1- denom * denom));
        }

        // ref: surveying.wb.psu.edu/sur351/georef/ellip3.htm (formula 6)
        const double meridRadiusOfCurvature( const double lat ) const {
            double eccSq = _eccentricity * _eccentricity;
            double den = 1 - eccSq * sin( lat ) * sin( lat );

            return _semiMajorAxis * (1-eccSq) / sqrt( den*den*den );
        }

    private:

        // private attributes
        unsigned short _code;
        double _semiMajorAxis;
        double _semiMinorAxis;
        double _flattening;
        double _eccentricity;
    };


}
