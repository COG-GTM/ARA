/**
 * Test stub for MMSTypes/GeoList.h (MPMS SDK).
 */
#pragma once

#include <cstring>
#include <string>
#include <vector>

#include "GeoPoint.h"

namespace MMS
{

class GeoList
{
public:
    using List = std::vector<GeoPoint>;

    /// Parses a WKT "LINESTRING (lon lat, lon lat, ...)" string.
    static List fromStr( const std::string & s )
    {
        List list;
        const char * p = std::strchr( s.c_str(), '(' );
        if ( !p )
            return list;
        ++p;
        while ( *p )
        {
            double lon = 0.0, lat = 0.0;
            if ( std::sscanf( p, "%lf %lf", &lon, &lat ) != 2 )
                break;
            list.emplace_back( lat, lon );
            const char * next = std::strchr( p, ',' );
            if ( !next )
                break;
            p = next + 1;
        }
        return list;
    }
};

} // namespace MMS
