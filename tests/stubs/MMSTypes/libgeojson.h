/**
 * Test stub for MMSTypes/libgeojson.h (MPMS SDK).
 */
#pragma once

#include "GeoList.h"
#include "GeoPoint.h"
#include "json.hpp"

namespace geojson
{

inline nlohmann::json Point( double lon, double lat )
{
    nlohmann::json geometry;
    geometry["type"]        = "Point";
    geometry["coordinates"] = nlohmann::json::array( { lon, lat } );
    return geometry;
}

inline nlohmann::json Feature( const nlohmann::json & geometry, const nlohmann::json & properties )
{
    nlohmann::json feature;
    feature["type"]       = "Feature";
    feature["geometry"]   = geometry;
    feature["properties"] = properties;
    return feature;
}

} // namespace geojson
