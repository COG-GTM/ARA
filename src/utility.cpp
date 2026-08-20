/**
 * This work contains valuable confidential and proprietary information.
 * Disclosure or reproduction without the written authorization of Neya
 * Systems, LLC is prohibited. This unpublished work by Neya Systems, LLC
 * is protected by the laws of the United States and other countries. If
 * publication of the work should occur, the following notice shall apply.
 *
 * Copyright (c) 2025 Neya Systems, LLC
 * All Rights Reserved
 *
 * The software/firmware is provided to you on an As-Is basis
 *
 * Delivered to the U.S. Government with Government Purpose Rights, as defined in
 * DFARS Part 252.227-7013 or 7014 (Feb 2014). This material may be
 * reproduced by or for the U.S Government pursuant to the copyright
 * license under the clause at DFARS 227-7203-5(a), DFARS 227.7103-5(a),
 * DFARS 252.227-7013(b)(1)(June 1995), DFARS 252.227-7014 (June 1995),
 * and FAR 52.227-14(a). Use of this work other than as specifically
 * authorized by the U.S. Government may violate any copyrights that
 * exist in this work.
 *
 * "WARNING - This file contains software and technical data whose export is
 * restricted by the Arms Export Control Act (Title 22, U.S.C., Sec
 * 2751, et seq.) or the Export Administration Act of 1979 (Title 50,
 * U.S.C., App. 2401 et seq.), as amended. Violations of these export
 * laws are subject to severe criminal penalties. Disseminate in
 * accordance with provisions of DoD Directive 5230.25."
 */
#include "utility.h"
#include "UTM.h"
#include <cstdio>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iostream>
//#include <typeinfo>

//using json_pair = nlohmann::detail::iteration_proxy_value<nlohmann::detail::iter_impl<nlohmann::basic_json<> > >;


double cartesianDistanceBetweenPoints(Waypoint pt1,Waypoint pt2)
{
	double deltaX,deltaY;
	double distance = 0.0;

	deltaX = pt1.m_x - pt2.m_x;
	deltaY = pt1.m_y - pt2.m_y;
	distance = sqrt((deltaX*deltaX) + (deltaY*deltaY));

	return distance;
}

Waypoint calculatePointOnLine(Waypoint pt1,Waypoint pt2,double distance)
{
	Waypoint retVal;
	double deltaX,deltaY;
	double alpha;
	double nextDx,nextDy;
	double slope = 0.0;

	deltaX = pt2.m_x - pt1.m_x;
	deltaY = pt2.m_y - pt1.m_y;

	if (deltaY == 0)
	{
		nextDy = 0;
		nextDx = distance;
	} else if (deltaX == 0)
	{
		nextDx = 0;
		nextDy = distance;
	} else
	{
		slope = deltaY / deltaX;
		nextDx = (distance / sqrt(1 + (slope * slope)));
		nextDy = slope * nextDx;
	}

	if (slope > 0)
	{
		if ((deltaX < 0) && (deltaY < 0))
		{
			nextDx = -nextDx;
			nextDy = -nextDy;
		}
	} else
	{
		if ((deltaX < 0) && (deltaY > 0))
		{
			nextDx = -nextDx;
			nextDy = -nextDy;
		}
	}


	retVal.m_x = pt1.m_x + nextDx;
	retVal.m_y = pt1.m_y + nextDy;
	retVal.zone = pt1.zone;

	return retVal;
}

MMS::GeoList::List parseWaypointWKTString( const std::string& s )
{
    std::string waypoint_string(toUpper(s));
    MMS::GeoList::List waypoint_list;
    MMS::GeoPoint waypoint;

    if( waypoint_string.find("POINT") != std::string::npos )
    {
        waypoint.setFromStr(waypoint_string);
        waypoint_list.push_back(waypoint);
    }
    else if( waypoint_string.find("LINESTRING") != std::string::npos )
        waypoint_list = MMS::GeoList::fromStr(waypoint_string);

    return waypoint_list;
}

namespace
{

// A GeoJSON position: [lon, lat] (optionally with altitude), both numeric.
bool isLonLatPair( const nlohmann::json& coord )
{
    return coord.is_array() && coord.size() >= 2 &&
           coord[0].is_number() && coord[1].is_number();
}

} // namespace

MMS::GeoList::List jsonToGeoList( nlohmann::json& geo_json )
{
    MMS::GeoList::List geo_list;
    MMS::GeoPoint waypoint;

	if( geo_json.empty() )
		return geo_list;

    if( !geo_json.is_object() || !geo_json["type"].is_string() )
    {
        std::cerr << RED << "[jsonToGeoList] - Malformed GeoJSON, expected an object with a string \"type\"" << NORMAL << std::endl;
        return geo_list;
    }

    if( equalsIgnoreCase(geo_json["type"], "Feature") )
    {
        geo_json = geo_json["geometry"];
        if( !geo_json.is_object() || !geo_json["type"].is_string() )
        {
            std::cerr << RED << "[jsonToGeoList] - Malformed GeoJSON Feature, expected a \"geometry\" object" << NORMAL << std::endl;
            return geo_list;
        }
    }

    if( equalsIgnoreCase(geo_json["type"], "Point") )
    {
        if( !isLonLatPair(geo_json["coordinates"]) )
        {
            std::cerr << RED << "[jsonToGeoList] - Malformed Point coordinates" << NORMAL << std::endl;
            return geo_list;
        }
        waypoint.setLon(geo_json["coordinates"][0]);
        waypoint.setLat(geo_json["coordinates"][1]);
        geo_list.push_back(waypoint);
    }
    else if( equalsIgnoreCase(geo_json["type"], "LineString") )
    {
        for( nlohmann::json& coord : geo_json["coordinates"] )
        {
            if( !isLonLatPair(coord) )
            {
                std::cerr << RED << "[jsonToGeoList] - Malformed LineString coordinates" << NORMAL << std::endl;
                return MMS::GeoList::List();
            }
            waypoint.setLon(coord[0]);
            waypoint.setLat(coord[1]);
            geo_list.push_back(waypoint);
        }
    }
    else if( equalsIgnoreCase(geo_json["type"], "Polygon") )
    {
        if( !geo_json["coordinates"].is_array() || geo_json["coordinates"].empty() )
        {
            std::cerr << RED << "[jsonToGeoList] - Malformed Polygon coordinates" << NORMAL << std::endl;
            return geo_list;
        }
        // Note this only parses the outer linear ring and omits the
        // inner linear ring defining any holes in the polygon
        for( nlohmann::json& coord : geo_json["coordinates"][0] )
        {
            if( !isLonLatPair(coord) )
            {
                std::cerr << RED << "[jsonToGeoList] - Malformed Polygon coordinates" << NORMAL << std::endl;
                return MMS::GeoList::List();
            }
            waypoint.setLon(coord[0]);
            waypoint.setLat(coord[1]);
            geo_list.push_back(waypoint);
        }
    }
    else
        std::cerr << RED << "[jsonToGeoList] - Unsupported GeoJSON Type \"" << geo_json["type"] << "\"" << NORMAL << std::endl;

    return geo_list;
}

MMS::GeoList::List jsonStringToGeoList( const std::string& s )
{
    nlohmann::json geo_json;

    try
    {
        geo_json = nlohmann::json::parse(s);
    }
    catch( const nlohmann::detail::parse_error& e )
    {
        std::cerr << RED << e.what() << NORMAL << std::endl;
        std::cerr << RED << "[jsonStringToGeoList] failed to parse JSON string \"" << s << "\"" << NORMAL << std::endl;
        return MMS::GeoList::List();
    }

    return jsonToGeoList(geo_json);
}

nlohmann::json geoPointToJson( const MMS::GeoPoint& point )
{
    nlohmann::json geoJson;
    geoJson["type"] = "Feature";
    geoJson["geometry"]["type"] = "Point";

    nlohmann::json coordinates = nlohmann::json::array();
    coordinates.push_back( point.getLon() );
    coordinates.push_back( point.getLat() );
    geoJson["geometry"]["coordinates"] = coordinates;

    return geoJson;
}

nlohmann::json geoListToJson( const MMS::GeoList::List& list )
{
    nlohmann::json geoJson;
    geoJson["type"] = "Feature";
    geoJson["geometry"]["type"] = "LineString";

    nlohmann::json coordinates = nlohmann::json::array();
    for ( const auto& point : list )
    {
        nlohmann::json pointJson = nlohmann::json::array();
        pointJson.push_back( point.getLon() );
        pointJson.push_back( point.getLat() );
        coordinates.push_back( pointJson );
    }
    geoJson["geometry"]["coordinates"] = coordinates;

    return geoJson;
}

double computeWaypointTraversalTime( MMS::GeoPoint position,
                                     nlohmann::json waypointsGeoJson,
                                     double maxSpeed )
{
    // Convert current position from lat/lon to UTM
    int utmZone;
    double x, y; // x : easting, y : northing
    LLtoUTM( position.getLat(), position.getLon(), x, y, utmZone );

    // Parse task waypoints
    MMS::GeoList::List waypoints = jsonToGeoList( waypointsGeoJson );

    // Initialize prev waypoint to current robot position to include travel
    // to first waypoint in total path distance
    Waypoint prevWaypoint{ x, y };

    // Loop through task waypoints computing total path distance
    double pathDistance{ 0. }; // meters
    for ( const auto& waypoint : waypoints )
    {
        // Convert current waypoint from lat/lon to UTM
        LLtoUTM( waypoint.getLat(), waypoint.getLon(), x, y, utmZone );
        Waypoint currWaypoint{ x, y };
        pathDistance += cartesianDistanceBetweenPoints( prevWaypoint, currWaypoint );
        prevWaypoint = currWaypoint;
    }

    return ( pathDistance / std::fabs( maxSpeed ) );
}

bool equalsIgnoreCase( const std::string& s1, const std::string& s2 )
{
    char c1, c2, upper;

    if( s1.length() != s2.length() )
        return false;

    upper = 'A' - 'a';

    for(int i = 0; i < s1.length(); i++)
    {
        c1 = s1[i];
        c2 = s2[i];

        if( c1 >= 'a' && c1 <= 'z' )
            c1 += upper;
        if( c2 >= 'a' && c2 <= 'z' )
            c2 += upper;

        if( c1 != c2 )
            return false;
    }

    return true;
}

std::string toUpper( const std::string& s )
{
    static const char upper('A' - 'a');
    std::string s_upper(s);

    for(int i = 0; i < s_upper.length(); i++)
        if( s_upper[i] >= 'a' && s_upper[i] <= 'z' )
            s_upper[i] += upper;

    return s_upper;
}

std::string trim(const std::string& s)
{
    int start, end, length;

    start = -1;
    for(int i = 0; i < s.length(); i++)
    {
        if( !isspace(s[i]) )
        {
            start = i;
            break;
        }
    }
    if( start < 0 )
        return "";

    end = s.length();
    for(int i = s.length() - 1; i >= 0; i--)
    {
        if( !isspace(s[i]) )
        {
            end = i;
            break;
        }
    }
    if( end == s.length() )
        return "";

    length = 1 + end - start;

    return s.substr(start, length);
}

std::map<std::string, std::string> parseConfigFile(const std::string& filename)
{
    FILE* file;
    int c = '?';
    std::string line;
    size_t breakpoint;
    std::string key, value;
    std::map<std::string, std::string> config;

    file = fopen(filename.c_str(), "r");

    if( file == nullptr )
    {
        std::cerr << "Failed to open config file \"" << filename << "\"" << std::endl;
        return config;
    }

    while( c != EOF )
    {
        c = fgetc(file);

        if( c != '\n' && c != EOF )
        {
            line += static_cast<char>( c );
            continue;
        }

        breakpoint = line.find("=");
        if( breakpoint != std::string::npos )
        {
            key = toUpper(trim( line.substr(0, breakpoint) ));
            std::string rawValue = trim(line.substr(breakpoint+1));

            // Only convert value to uppercase if it's not a filename
            std::ifstream ifs{rawValue};
            value = ifs.fail() ? toUpper(rawValue) : rawValue;
            config[key] = value;
        }

        line = "";
    }
    fclose(file);

    return config;
}

void writeConfigFile(const std::string& filename, const std::map<std::string, std::string>& config)
{
    FILE *file;

    file = fopen(filename.c_str(), "w");

    if( file == nullptr )
    {
        std::cerr << "Failed to open file \"" << filename << "\" for writing" << std::endl;
        return;
    }

    for( const std::pair<std::string,std::string> entry : config )
        fputs( (entry.first + " = " + entry.second + "\n").c_str(), file );
    fclose(file);
}

unsigned short& jausSystemID( int& jaus_id )
{
    return ((unsigned short*)&jaus_id)[0];
}

unsigned char& jausNodeID( int& jaus_id )
{
    return ((unsigned char*)&jaus_id)[2];
}

unsigned char& jausComponentID( int& jaus_id )
{
    return ((unsigned char*)&jaus_id)[3];
}

int jausID( unsigned short system_id, unsigned char node_id, unsigned char component_id )
{
    int jaus_id;

    jausSystemID(jaus_id) = system_id;
    jausNodeID(jaus_id) = node_id;
    jausComponentID(jaus_id) = component_id;

    return jaus_id;
}
