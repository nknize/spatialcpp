// ellipsoid.cpp

/**
 *    Copyright (C) 2012 Thermopylae Sciences + Technology
 *
 *    This program is free software: you can redistribute it and/or modify
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

#include "ellipsoid.h"

namespace spatial {

    /**
     * Defines a ellipsoidal datum
     */
    typedef struct {
        unsigned short code;
    //    char *USGSName;
        double semiMajorAxis;
        double invFlattening;
        string fullName;
    } EllipsoidDefinition;

    /**
     * Easily accessible array of geodetic system definitions
     * @REF: worldwind31.arc.nasa.gov
     */
    static const EllipsoidDefinition ellipsoidList[] = {
        { 7001, 6377563.396, 299.3249646, "Airy 1830" },
        { 7002, 6377340.189, 299.3249646, "Airy Modified 1849"},
        { 7003, 6378160.0, 298.25, "Australian National Spheroid" },
        { 7004, 6377397.155, 299.1528128, "Bessel 1841" },
        { 7005, 6377492.018, 299.1528128, "Bessel Modified" },
        { 7006, 6377483.865, 299.1528128, "Bessel Namibia" },
        { 7007, 20926348.0, 294.26068, "Clarke 1858" },
        { 7008, 6378206.4, 294.979, "Clarke 1866" },
        { 7009, 20926631.531, 294.97870, "Clarke 1866 Michigan" },
        { 7010, 6378300.789, 293.466279237, "Clarke 1880 (Benoit)" },
        { 7011, 6378249.2, 293.466481398, "Clarke 1880 (IGN)" },
        { 7012, 6378249.145, 293.465, "Clarke 1880 (RGS)" },
        { 7013, 6378249.145, 293.4663077, "Clarke 1880 (Arc)" },
        { 7014, 6378249.2, 293.46598, "Clarke 1880 (SGA 1922)" },
        { 7015, 6377276.345, 300.8017, "Everest 1830 (1937 Adjustment)" },
        { 7016, 6377298.556, 300.8017, "Everest 1830 (1967 Definition)" },
        { 7018, 6377304.063, 300.8017, "Everest 1830 Modified" },
        { 7019, 6378137.0, 298.257222101, "GRS 1980" },
        { 7020, 6378200.0, 298.3, "Helmert 1906" },
        { 7021, 6378160.0, 298.247, "Indonesian National Spheroid" },
        { 7022, 6378388.0, 297.0, "International 1924" },
        { 7024, 6378245.0, 298.3, "Krassowsky 1940" },
        { 7025, 6378145.0, 298.25, "NWL 9D" },
        { 7027, 6376523.0, 308.64, "Plessis 1817" },
        { 7028, 6378298.3, 294.73, "Struve 1860" },
        { 7029, 6378300.0, 296.0, "War Office"},
        { 7030, 6378137.0, 298.257223563, "WGS 84" },
        { 7031, 6378137.0, 298.257223563, "GEM 10C" },
        { 7032, 6378136.2, 298.257223563, "OSU86F" },
        { 7033, 6378136.3, 298.257223563, "OSU91A" },
        { 7034, 20926202.0, 293.465, "Clarke 1880" },
        { 7035, 6371000.0, 0.0, "Sphere" }, // Flattening is infinite
        { 7036, 6378160.0, 298.247167427, "GRS 1967" },
        { 7041, 6378135.0, 298.257, "Average Terrestrial System 1977" },
        { 7042, 20922931.8, 300.8017, "Everest (1830 Definition)" },
        { 7043, 6378135.0, 298.26, "WGS 72" },
        { 7044, 6377301.243, 300.8017255, "Everest 1830 (1962 Definition)" },
        { 7045, 6377299.151, 300.8017255, "Everest 1830 (1975 Definition)" },
        { 7046, 6377397.155, 299.1528128, "Bessel Namibia (GLM)" },
        { 7047, 6370997.0, 0.0, "GRS 1980 Authalic Sphere" }, // Flattening INF
        { 7048, 6371007.0, 0.0, "GRS 1980 Authalic Sphere" },
        { 7049, 6378140.0, 298.257, "Xian 1980" },
        { 7050, 6378160.0, 298.25, "GRS 1967 Modified" },
        { 7051, 6377019.27, 300.0, "Danish 1876" },
        { 7052, 6370997.0, 0.0, "Clarke 1866 Authalic Sphere" }, // Flattening INF
        { 7053, 6378270.0, 297.0, "Hough 1960" },
        { 7054, 6378136.0, 298.257839303, "PZ-90" },
        { 7055, 20926202.0, 293.4663077, "Clarke 1880 (international foot)" },
        { 7056, 6377295.664, 300.8017, "Everest 1830 (RSO 1969)" },
        { 7057, 6371228.0, 0.0, "International 1924 Authalic Sphere" },
        { 7058, 6378273.0, 298.780735574, "Hughes 1980" },
        { 7059, 6378137.0, 0.0, "Popular Visualisation Sphere"}, // usually used for web mapping
    };


    /**
     *  Initializes the ellipsoid datum
     */
    void Ellipsoid::initialize() {
        unsigned short n = sizeof(ellipsoidList)/
                           sizeof(EllipsoidDefinition);

        bool initialized = false;
        for( unsigned short i=0; i<n; ++i ) {
            if( this->_code == ellipsoidList[i].code ) {
                // compute the attribute values
                this->_flattening = 1.0 / ellipsoidList[i].invFlattening;
                this->_semiMajorAxis = ellipsoidList[i].semiMajorAxis;
                this->_semiMinorAxis = this->_semiMajorAxis * (1.0 - this->_flattening);
                this->_eccentricity = sqrt( (2.0-this->_flattening)*this->_flattening );
                initialized = true;

                break;
            }
        }
        
        uassert(1000, "Ellipsoid::initialize(): Failed to initialize ellipsoid datum", initialized);
    }

}

