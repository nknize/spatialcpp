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

    /**
     * Ellipsoid class defining a projection
     *
     * This class is a thread-safe singleton
     */
    class Ellipsoid {
    protected:

        /**
         * Parameter structure for converting this projection definition to
         * UTM
         */
        typedef struct {
            // lat lon to utm params
            double rm;
            double n;
            double rho; // merid radius of curvature
            double nu; // prime vertical radius of curvature
            double S;
            double A0, B0, C0, D0, E0;
            double p;
            double K1, K2, K3, K4, K5, A6;
            static constexpr const double sin1 = 4.84814e-06; // legacy factor

            // utm to lat lon params
            double arc;
            double mu;
            double ei;
            double ca, cb, cc, cd;
            double n0;
            double r0;
            double _a1, _a2, _a3;
            double dd0;
            double t0;
            double Q0;
            double lof1, lof2, lof3;
            double phi1;
            double fact1, fact2, fact3, fact4;
            double zoneCM;
            static constexpr const double k0 = 0.9996; // central meridian scale factor for UTM
        } UTMParams;

        void initializeUTMtoLLParams( const CartesianPoint3D& utmPt );
        void initializeLLtoUTMParams( const GeoPointRad3D& geoPt );

    public:

        // get the singleton instance
        static Ellipsoid& get(const unsigned int code = 7030) {
          std::call_once(_onceFlag, [code] {
            _instance.reset(new Ellipsoid(code));
          });

          Ellipsoid& e = *_instance.get();
          if(code != e._code) {
              e._code = code;
              e.initialize();
          }
          return e;
        }

        // - conversions 
        //      global assumptions: 
        //          *this* object is not modified, new objects are returned
        //          angles are in radians except 
        
        // converts from earth-centered earth-fixed cartesian to lat,lon,alt spherical
        GeoPointRad3D&& ecfToLLARad( const CartesianPoint3D& ecfPt ) const;
        GeoPointRad3D&& enuToLLA( const CartesianPoint3D& enuPt, const GeoPointRad3D& refPt ) const;
        GeoPointRad3D&& utmToLLA( const string& utmPt ) const;
        void mgrsToLatLonAlt( const string& mgrsPt, GeoPointRad3D &geoPt ) const;

        CartesianPoint3D&& llaToECF( const GeoPointRad3D &llaPt ) const;
        void enuToECF( const CartesianPoint3D& enuPt, const GeoPointRad3D& refPt, 
                       CartesianPoint3D &ecfPt ) const;
        void utmToECF( const CartesianPoint3D& utmPt, CartesianPoint3D &ecfPt ) const;
        void mgrsToECF( const string& mgrsPt, CartesianPoint3D &ecfPt ) const;

        void latLonAltToENU( const GeoPointRad3D& llaPt, const GeoPointRad3D& refPt,
                             CartesianPoint3D &enuPt ) const;
        void ecfToENU( const CartesianPoint3D& ecfPt, const GeoPointRad3D& refPt, 
                       CartesianPoint3D &enuPt ) const;
        // utm
        void latLonAltToUTM( const GeoPointRad3D& llaPt, string &utmPt );
        
        // mgrs
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
        GeoPointDeg3D&& ecfToLLADeg( const CartesianPoint3D& ecfPt ) const {
 //         cout << "CALLING ecfTOLLARad with " << ecfPt.toString() << endl;
          GeoPointRad3D rad = ecfToLLARad( ecfPt );
            GeoPointDeg3D llaDeg;
 //           cout << "CALLING TRANSFORM WITH " << rad.toString() << endl;
            transform(rad, llaDeg);

//            cout << llaDeg.toString() << endl;

            return std::move(llaDeg);
        }
        void enuToLatLonAltDeg( const CartesianPoint3D& enuPt, const GeoPointRad3D& refPt,
                                GeoPointDeg3D &llaDeg) const {
            GeoPointRad3D rad;
            rad = enuToLLA( enuPt, refPt );
            transform(rad, llaDeg);
        }
        void utmToLatLonAltDeg( const string& utmPt,
                                GeoPointDeg3D &llaDeg) const {
            GeoPointRad3D rad;
            rad = utmToLLA( utmPt );
            transform(rad, llaDeg);
        }
        void mgrsToLatLonAltDeg( const string& mgrsPt,
                                 GeoPointDeg3D& llaDeg) const {
            GeoPointRad3D llaRad;
            mgrsToLatLonAlt( mgrsPt, llaRad );
            transform(llaRad, llaDeg);
        }
        CartesianPoint3D&& llaDegToECF( const GeoPointDeg3D& llaDeg ) const {
            GeoPointRad3D llaRad;
            transform(llaDeg, llaRad);
 //           cout << llaDeg.toString() << "----- lla in degs" << endl;
 //           cout << llaRad.toString() << "-----lla in rads" << endl;
            CartesianPoint3D ecef = llaToECF( llaRad );
 //           cout << ecef.toString() << " ------ecef result prior to move" << endl;
            return std::move(ecef);
        }
        void latLonAltDegToUTM( const GeoPointDeg3D& llaDeg,
                                string& utm) {
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
        static constexpr const double _ITERATION_THRESHOLD = 4.8481368110953599e-08;

        // print this ellipsoid object
        // @TODO: Replace this operation with a string buffer
        friend ostream& operator << ( ostream& os, const Ellipsoid& ellipsoid );

        // ref: surveying.wb.psu.edu/sur351/georef/ellip3.htm (formula 7)
        const double primeVertRadiusOfCurvature( const double lat ) const {
            const double denom = (_eccentricity*_eccentricity) * (sin(lat)*sin(lat));
            return (_semiMajorAxis / sqrt( 1 - denom ));
        }

        // ref: surveying.wb.psu.edu/sur351/georef/ellip3.htm (formula 6)
        const double meridRadiusOfCurvature( const double lat ) const {
            double eccSq = _eccentricity * _eccentricity;
            double den = 1 - eccSq * sin( lat ) * sin( lat );

            return _semiMajorAxis * (1-eccSq) / sqrt( den*den*den );
        }

    private:

        static std::unique_ptr<Ellipsoid> _instance;
        static std::once_flag _onceFlag;

        /**
         * Constructor defaults to code 7030,
         * which is the EPG code for the WGS84 ellipsoid
         */
        Ellipsoid( const unsigned short code = 7030 ) : _code(code) { initialize(); }

        // private attributes
        unsigned short _code;
        double _semiMajorAxis;
        double _semiMinorAxis;
        double _flattening;
        double _eccentricity;
        double _ee;

        UTMParams _utmParams;


        class Digraphs {
        public:
            Digraphs() : digraph1(), digraph2() {
                digraph2[0] = string("V");
                // loop through characters 'A' through 'Z' in the 
                // ASCII table using decimal variable 'c'
                for(int c=65, i=0, j=0; c<=90; ++c) {
                    // skip 'I' and 'O' for digraph 1 and 2
                    if( c!=(int)'I' && c!=(int)'O') {
                        digraph1[++i] = string(1, (char)c);
                        digraph1Array[i-1] = string(1, (char)c);
                        // skip 'W', 'X', 'Y', 'Z' for digraph2
                        if( c!=(int)'W' && c!=(int)'X' &&
                            c!=(int)'Y' && c!=(int)'Z') 
                            digraph2[++j] = string(1, (char)c);
                            digraph2Array[j-1] = string(1, (char)c);
                    }
                }
            }

            int getDigraph1Index(string &letter) {
                for(int i=0; i<24; ++i)
                    if(!digraph1Array[i].compare(letter)) return i+1;
                return -1;
            }

            int getDigraph2Index(string &letter) {
                for(int i=0; i<21; ++i)
                    if(!digraph2Array[i].compare(letter)) return i;
                return -1;
            }

            string getDigraph1( int lonZone, double easting ) {
                double a1 = 8.0 * ((lonZone-1) % 3) + 1.0;
                double a2 = a1 + ((int)(easting/100000))-1.0;
                return digraph1.at( (int)floor(a2) );
            }

            string getDigraph2( int lonZone, double northing ) {
                double a2 = 1.0 + 5.0 * ((lonZone-1)%2);
                double a4 = (a2 + ((int)(northing/100000)));
                a4 = (int)floor((a2 + ((int)(northing/100000.0)))) % 20;
                if(a4<0) a4+=19.0;
                return digraph2.at( (int)a4 );
            }

        private: 
            boost::unordered_map<int, string> digraph1;
            boost::unordered_map<int, string> digraph2;
            string digraph1Array[24];
            string digraph2Array[21];

        };

/*
        class LatZones {
        public:

        private:
            static const int degrees[22]; = { -90, -84, -72, -64, -56, -48, -40, -32, -24, -16,
                    -8, 0, 8, 16, 24, 32, 40, 48, 56, 64, 72, 84 };
            static char* letters = { 'A', 'C', 'D', 'E', 'F', 'G', 'H', 'J', 'K',
                    'L', 'M', 'N', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Z'};
        

        };

        const int LatZones::degrees[22] = { -90, -84, -72, -64, -56, -48, -40, -32, -24, -16,
                -8, 0, 8, 16, 24, 32, 40, 48, 56, 64, 72, 84 };

*/

    };

    inline Ellipsoid& ellipsoid(const unsigned int code = 7030) {
      return Ellipsoid::get(code);
    }
} // spatial namespace
