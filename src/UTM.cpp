/*
This work contains valuable confidential and proprietary information.
Disclosure or reproduction without the written authorization of Neya
Systems, LLC is prohibited. This unpublished work by Neya Systems, LLC
is protected by the laws of the United States and other countries. If
publication of the work should occur, the following notice shall apply.

Copyright (c) 2025 Neya Systems, LLC
All Rights Reserved

The software/firmware is provided to you on an As-Is basis

Delivered to the U.S. Government with Government Purpose Rights, as defined in
DFARS Part 252.227-7013 or 7014 (Feb 2014). This material may be
reproduced by or for the U.S Government pursuant to the copyright
license under the clause at DFARS 227-7203-5(a), DFARS 227.7103-5(a),
DFARS 252.227-7013(b)(1)(June 1995), DFARS 252.227-7014 (June 1995),
and FAR 52.227-14(a). Use of this work other than as specifically
authorized by the U.S. Government may violate any copyrights that
exist in this work.

"WARNING - This file contains software and technical data whose export is
restricted by the Arms Export Control Act (Title 22, U.S.C., Sec
2751, et seq.) or the Export Administration Act of 1979 (Title 50,
U.S.C., App. 2401 et seq.), as amended. Violations of these export
laws are subject to severe criminal penalties. Disseminate in
accordance with provisions of DoD Directive 5230.25."
*/

#include "UTM.h"
#include <gdal/ogr_geometry.h>
#include <gdal/ogr_spatialref.h>
#include <iostream>

static constexpr double RADIANS_PER_DEGREE = M_PI / 180.0;

int UTMZone( double latitude, double longitude )
{
    return ( ( (int)( longitude + 180 ) ) / 6 ) + 1;
}

void LLtoUTM( double Lat, double Long, double& UTMEasting, double& UTMNorthing, int& UTMZone )
{
    // Acquired from Robot Localization / Navsat Conversion
    double a          = 6378137.0;                       // major axis;
    double eccSquared = ( 0.0818191908 * 0.0818191908 ); // e^2;
    double k0         = 0.9996;                          // scale factor;

    double LongOrigin;
    double eccPrimeSquared;
    double N, T, C, A, M;

    // Make sure the longitude is between -180.00 .. 179.9
    double LongTemp = ( Long + 180 ) - static_cast<int>( ( Long + 180 ) / 360 ) * 360 - 180;

    double LatRad  = Lat * RADIANS_PER_DEGREE;
    double LongRad = LongTemp * RADIANS_PER_DEGREE;
    double LongOriginRad;
    int ZoneNumber;

    ZoneNumber = static_cast<int>( ( LongTemp + 180 ) / 6 ) + 1;

    if ( Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0 )
        ZoneNumber = 32;

    // Special zones for Svalbard
    if ( Lat >= 72.0 && Lat < 84.0 )
    {
        if ( LongTemp >= 0.0 && LongTemp < 9.0 )
            ZoneNumber = 31;
        else if ( LongTemp >= 9.0 && LongTemp < 21.0 )
            ZoneNumber = 33;
        else if ( LongTemp >= 21.0 && LongTemp < 33.0 )
            ZoneNumber = 35;
        else if ( LongTemp >= 33.0 && LongTemp < 42.0 )
            ZoneNumber = 37;
    }
    // +3 puts origin in middle of zone
    LongOrigin    = ( ZoneNumber - 1 ) * 6 - 180 + 3;
    LongOriginRad = LongOrigin * RADIANS_PER_DEGREE;

    // Compute the UTM Zone from the latitude and longitude
    UTMZone = ZoneNumber;

    eccPrimeSquared = ( eccSquared ) / ( 1 - eccSquared );

    N = a / sqrt( 1 - eccSquared * sin( LatRad ) * sin( LatRad ) );
    T = tan( LatRad ) * tan( LatRad );
    C = eccPrimeSquared * cos( LatRad ) * cos( LatRad );
    A = cos( LatRad ) * ( LongRad - LongOriginRad );

    M = a * ( ( 1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64 -
                5 * eccSquared * eccSquared * eccSquared / 256 ) *
                  LatRad -
              ( 3 * eccSquared / 8 + 3 * eccSquared * eccSquared / 32 +
                45 * eccSquared * eccSquared * eccSquared / 1024 ) *
                  sin( 2 * LatRad ) +
              ( 15 * eccSquared * eccSquared / 256 + 45 * eccSquared * eccSquared * eccSquared / 1024 ) *
                  sin( 4 * LatRad ) -
              ( 35 * eccSquared * eccSquared * eccSquared / 3072 ) * sin( 6 * LatRad ) );

    UTMEasting = static_cast<double>(
        k0 * N *
            ( A + ( 1 - T + C ) * A * A * A / 6 +
              ( 5 - 18 * T + T * T + 72 * C - 58 * eccPrimeSquared ) * A * A * A * A * A / 120 ) +
        500000.0 );

    UTMNorthing =
        static_cast<double>( k0 * ( M + N * tan( LatRad ) *
                                            ( A * A / 2 + ( 5 - T + 9 * C + 4 * C * C ) * A * A * A * A / 24 +
                                              ( 61 - 58 * T + T * T + 600 * C - 330 * eccPrimeSquared ) * A *
                                                  A * A * A * A * A / 720 ) ) );

    if ( Lat < 0 )
    {
        // 10000000 meter offset for southern hemisphere
        UTMNorthing += 10000000.0;
    }
}

void UTMtoLL(
    int zone, double easting, double northing, double& latitude, double& longitude, bool northern_hemisphere )
{
    OGRSpatialReference source, target;
    OGRCoordinateTransformation* transform( nullptr );
    double x, y;

    // set up source (UTM)
    source.SetProjCS( "UTM 17 (WGS84) in northern hemisphere." );
    source.SetWellKnownGeogCS( "WGS84" );
    source.SetUTM( zone, northern_hemisphere );

    // set up target (lat/lon)
    target.SetGeogCS( "My geographic coordinate system",
                      "WGS_1984",
                      "My WGS84 Spheroid",
                      SRS_WGS84_SEMIMAJOR,
                      SRS_WGS84_INVFLATTENING,
                      "Greenwich",
                      0.0 );

    // create transform
    transform = OGRCreateCoordinateTransformation( &source, &target );

    x = easting;
    y = northing;

    // transform
    if ( transform == nullptr || !transform->Transform( 1, &x, &y ) )
        std::cout << "Failed to convert UTM zone " << zone << " easting/northing " << easting << "/"
                  << northing << " to UTM" << std::endl;
    else
    {
        longitude = x;
        latitude  = y;
    }

    // cleanup
    if ( transform )
        OCTDestroyCoordinateTransformation( transform );
}

double headingFromLLPoints( double latitude_a, double longitude_a, double latitude_b, double longitude_b )
{
    latitude_a *= ( 180.0 / M_PI );
    latitude_b *= ( 180.0 / M_PI );
    longitude_a *= ( 180.0 / M_PI );
    longitude_b *= ( 180.0 / M_PI );

    double X = cos( latitude_b ) * sin( longitude_b - longitude_a );
    double Y = cos( latitude_a ) * sin( latitude_b ) -
               sin( latitude_a ) * cos( latitude_b ) * cos( longitude_b - longitude_a );

    return -atan2( X, Y ) + M_PI / 2.0;
}

void nextWaypoint( double start_latitude,
                   double start_longitude,
                   double goal_latitude,
                   double goal_longitude,
                   double& easting,
                   double& northing,
                   int& zone )
{
    int start_zone, goal_zone;
    double border_latitude, border_longitude;
    double d_lat, d_lon;
    bool same_hemisphere;

    // Get UTM zones for start and goal points
    start_zone      = UTMZone( start_latitude, start_longitude );
    goal_zone       = UTMZone( goal_latitude, goal_longitude );
    same_hemisphere = ( start_latitude * goal_latitude ) >= 0;

    // If start and goal points are in the same zone and hemisphere, just convert
    // the goal to UTM
    if ( ( start_zone == goal_zone ) && same_hemisphere )
    {
        LLtoUTM( goal_latitude, goal_longitude, easting, northing, zone );
        return;
    }

    // Calculate difference in longitudes and latitudes
    d_lat = goal_latitude - start_latitude;
    d_lon = goal_longitude - start_longitude;

    // If the difference in longitudes is greater than 180 degrees, it's faster to
    // go the other way
    if ( fabs( d_lon ) > 180.0 )
    {
        if ( d_lon > 0 )
            d_lon -= 360.0;
        else
            d_lon += 360.0;
    }

    // Check if the start and goal points are in different hemispheres
    if ( !same_hemisphere )
    {
        // Calculate the point on the equator in the direction of the goal, just
        // inside the hemisphere of the start point
        if ( start_latitude < 0.0 )
            border_latitude = -0.0001;
        else
            border_latitude = 0.0001;
        border_longitude = start_longitude + ( d_lon * ( ( border_latitude - start_latitude ) / d_lat ) );

        // If the point at the equator in the direction of the goal is in the same
        // zone as the start zone, cross the equator first
        if ( UTMZone( border_latitude, border_longitude ) == start_zone )
        {
            // If the start is close to the equator, just cross it
            if ( fabs( start_latitude - border_latitude ) <= 0.00011 )
            {
                LLtoUTM( start_latitude, start_longitude, easting, northing, zone );
                if ( goal_latitude > 0 )
                    northing += 12;
                else
                    northing -= 12;
                return;
            }

            LLtoUTM( border_latitude, border_longitude, easting, northing, zone );
            return;
        }
    }

    // Calculate the longitude of the zone border in the direction of the goal
    if ( d_lon > 0 )
        border_longitude = ( ( ( start_zone - 1 + 1 ) * 6.0 ) - 180.0 );
    else
        border_longitude = ( ( ( start_zone - 1 ) * 6.0 ) - 180.0 );

    // If the start longitude is within 0.0001 degrees (roughly 11 meters at the
    // equator, less at higher/lower latitudes) of the zone border, just cross it
    if ( fabs( start_longitude - border_longitude ) <= 0.00011 )
    {
        LLtoUTM( start_latitude, start_longitude, easting, northing, zone );
        if ( d_lon > 0 )
            easting += 12;
        else
            easting -= 12;
        return;
    }

    // Move border longitude to just inside the zone border
    if ( d_lon > 0 )
        border_longitude -= 0.0001;
    else
        border_longitude += 0.0001;

    // Calculate the latitude of the desired zone border point (linear
    // interpolation of lat/lon coordinates creates non-optimal path, but it's
    // close enough)
    border_latitude = start_latitude + ( d_lat * ( ( border_longitude - start_longitude ) / d_lon ) );

    // Convert border point to UTM
    LLtoUTM( border_latitude, border_longitude, easting, northing, zone );
}
